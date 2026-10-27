#include "roar_localization/wio.hpp"
#include <cmath>

namespace roar {

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / reset
// ─────────────────────────────────────────────────────────────────────────────
WIO::WIO(const WIOParams& params)
    : params_(params)
{
    reset();
}

void WIO::reset()
{
    prev_pulses_L_ = 0.0;
    prev_pulses_R_ = 0.0;
    prev_yaw_      = 0.0;
    first_call_    = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// compute()
//
// Called at ~100 Hz (every IMU message).
//
// Five steps:
//   1. Encoder pulses → individual wheel speeds v_L, v_R → average vx
//   2. BNO055 quaternion → R → roll / pitch / yaw
//   3. Outlier / spike rejection
//   4. Mid-point integration for dp_world
//   5. Pack and return WIOOutput
//
// CHANGE from original:
//   v_L and v_R are now stored separately in the output so the node can
//   compute slip_ratio = |v_R - v_L| / (|v_R| + |v_L| + ε) for the WO-FIS.
//   On any early-return (invalid reading, first call, spike), v_L and v_R
//   are set to 0.0 so the FIS sees zero slip — the safest fallback.
// ─────────────────────────────────────────────────────────────────────────────
WIOOutput WIO::compute(double      pulses_L,
                       double      pulses_R,
                       const Quat& q_imu,
                       double      dt)
{
    WIOOutput out;
    out.valid = true;
    out.v_L   = 0.0;    // default: zero slip on invalid / first call
    out.v_R   = 0.0;

    // ── Step 1: Encoder deltas ────────────────────────────────────────────────
    const double delta_L = pulses_L - prev_pulses_L_;
    const double delta_R = pulses_R - prev_pulses_R_;

    // On the very first call we have no previous state.
    // Return a zero-motion output so the filter starts cleanly.
    if (first_call_) {
        prev_pulses_L_ = pulses_L;
        prev_pulses_R_ = pulses_R;
        first_call_    = false;

        out.R        = quat_to_rot(q_imu);
        out.vx       = 0.0;
        out.dp_world = Vec3::Zero();
        Vec3 euler   = quat_to_euler(q_imu);
        out.yaw      = euler(2);
        out.pitch    = euler(1);
        prev_yaw_    = out.yaw;
        return out;
    }

    // ── Step 2: Spike rejection ───────────────────────────────────────────────
    // If a single step has an implausible pulse count the encoder missed an
    // interrupt or a wire glitched. Output zero motion; do NOT advance counts.
    if (std::abs(delta_L) > params_.max_delta_pulses ||
        std::abs(delta_R) > params_.max_delta_pulses)
    {
        out.valid    = false;
        out.R        = quat_to_rot(q_imu);
        out.vx       = 0.0;
        out.dp_world = Vec3::Zero();
        Vec3 euler   = quat_to_euler(q_imu);
        out.yaw      = euler(2);
        out.pitch    = euler(1);
        // Do NOT update prev_pulses on a spike.
        return out;
    }

    // ── Step 3: Pulses → individual wheel speeds ──────────────────────────────
    //   one full revolution = pulses_per_rev counts = 2π × wheel_radius metres
    //
    // NOTE on encoder format:
    //   - If your driver publishes RADIANS: set pulses_per_rev = 2π (6.2832).
    //     Then metres_per_pulse = 2π×r / 2π = r, and
    //     speed = delta_rad × r / dt — exactly correct.
    //   - If your driver publishes RAW COUNTS: set pulses_per_rev to your
    //     encoder's actual PPR × quadrature_factor.
    const double metres_per_pulse = (2.0 * M_PI * params_.wheel_radius)
                                    / params_.pulses_per_rev;

    const double v_L = (delta_L * metres_per_pulse) / dt;
    const double v_R = (delta_R * metres_per_pulse) / dt;

    // Average forward speed — yaw comes from BNO055, not from the differential
    // (v_R - v_L), because differential speed is unreliable on rough terrain.
    const double vx = 0.5 * (v_L + v_R);

    // Speed sanity check: reject if above physical maximum.
    if (std::abs(vx) > params_.max_vx) {
        out.valid    = false;
        out.R        = quat_to_rot(q_imu);
        out.vx       = 0.0;
        out.dp_world = Vec3::Zero();
        Vec3 euler   = quat_to_euler(q_imu);
        out.yaw      = euler(2);
        out.pitch    = euler(1);
        prev_pulses_L_ = pulses_L;
        prev_pulses_R_ = pulses_R;
        return out;
    }

    // Reading was valid. Update stored counts.
    prev_pulses_L_ = pulses_L;
    prev_pulses_R_ = pulses_R;

    // ── Step 4: BNO055 quaternion → R, Euler ─────────────────────────────────
    const Mat3 R     = quat_to_rot(q_imu);
    const Vec3 euler = rot_to_euler(R);
    const double pitch = euler(1);
    const double yaw   = euler(2);

    // ── Step 5: Mid-point yaw integration ────────────────────────────────────
    // Halves the integration error when the rover is turning.
    const double d_yaw   = wrap_angle(yaw - prev_yaw_);
    const double avg_yaw = prev_yaw_ + 0.5 * d_yaw;
    prev_yaw_ = yaw;

    // ── Step 6: Dead-reckoning dp in world frame ──────────────────────────────
    // dp_z is non-zero on slopes.
    const double dp_x = vx * std::cos(avg_yaw) * dt;
    const double dp_y = vx * std::sin(avg_yaw) * dt;
    const double dp_z = vx * std::sin(pitch)   * dt;

    // ── Pack output ───────────────────────────────────────────────────────────
    out.R        = R;
    out.vx       = vx;
    out.v_L      = v_L;   // ← NEW: left wheel speed for slip ratio
    out.v_R      = v_R;   // ← NEW: right wheel speed for slip ratio
    out.dp_world = Vec3(dp_x, dp_y, dp_z);
    out.yaw      = yaw;
    out.pitch    = pitch;
    out.valid    = true;

    return out;
}

} // namespace roar
