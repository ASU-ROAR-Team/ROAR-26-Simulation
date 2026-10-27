#pragma once

#include "roar_localization/math_utils.hpp"

namespace roar {

// ─────────────────────────────────────────────────────────────────────────────
// WIOParams
// Physical constants of the rover's drivetrain.
// Set these once at startup from your URDF / hardware spec.
// ─────────────────────────────────────────────────────────────────────────────
struct WIOParams
{
    // Wheel radius [m]. Measure the actual loaded wheel (with rover weight on).
    double wheel_radius   = 0.1;

    // Distance between left and right wheel contact points [m].
    double wheelbase      = 0.5;

    // Encoder pulses per full wheel revolution (both edges of both channels).
    // e.g. a 500 PPR quadrature encoder = 2000 counts per revolution.
    // If your driver publishes in RADIANS, set this to 2π (= 6.2832).
    double pulses_per_rev = 2000.0;

    // Maximum plausible forward speed [m/s].
    // Readings above this are treated as encoder glitch and rejected.
    double max_vx         = 3.0;

    // Maximum plausible encoder delta per step.
    // For pulse-count mode: ~500 pulses/step at 100 Hz.
    // For radian mode:      set to ~π/4 (45° per 10 ms = very fast).
    double max_delta_pulses = 500.0;
};

// ─────────────────────────────────────────────────────────────────────────────
// WIOOutput
// Everything the IESKF predict step and WO-FIS need from WIO.
// ─────────────────────────────────────────────────────────────────────────────
struct WIOOutput
{
    Mat3   R;           // rotation matrix body→world  (from BNO055 quaternion)
    double vx;          // forward speed [m/s]         (from encoders, average)
    Vec3   dp_world;    // position delta [m]           (world frame, dead reckoning)
    double yaw;         // current yaw [rad]            (for debugging / logging)
    double pitch;       // current pitch [rad]          (for slope dp_z)

    // ── NEW: individual wheel speeds (needed by WO-FIS for slip ratio) ────────
    double v_L;         // left  wheel surface speed [m/s]
    double v_R;         // right wheel surface speed [m/s]
    // slip_ratio = |v_R - v_L| / (|v_R| + |v_L| + ε)
    // Computed from v_L/v_R in the node (or via fuzzy::computeSlipRatio()).

    bool   valid;       // false if encoders produced implausible readings
};

// ─────────────────────────────────────────────────────────────────────────────
// WIO — Wheel-Inertial Odometry
//
// Combines wheel encoder speed with BNO055 orientation to produce a
// dead-reckoning position delta for the IESKF predict step, and individual
// wheel speeds for the WO-FIS slip detector.
//
// Key design choices:
//   - Forward speed vx comes from encoders (more reliable than integrating a_m).
//   - Heading comes from BNO055 yaw (more reliable than encoder differential
//     on rough/slippery outdoor terrain).
//   - dp_z is non-zero when pitch is non-zero (handles slopes).
//   - Encoder spikes and implausible readings are rejected.
//   - v_L and v_R are now exposed for slip ratio computation.
//
// Usage:
//   WIO wio(params);
//   WIOOutput out = wio.compute(pulses_L, pulses_R, q_imu, dt);
//   double slip = fuzzy::computeSlipRatio(out.v_L, out.v_R);
//   ieskf.predict(a_m, omega_m, out.R, out.vx, out.dp_world, dt, k_Q);
// ─────────────────────────────────────────────────────────────────────────────
class WIO
{
public:
    explicit WIO(const WIOParams& params = WIOParams{});

    // Reset internal state (call on startup or after a long pause).
    void reset();

    // compute() — call once per IMU message (~100 Hz).
    //
    // pulses_L  : raw encoder pulse count from left  wheel (cumulative or delta)
    // pulses_R  : raw encoder pulse count from right wheel (cumulative or delta)
    // q_imu     : BNO055 orientation quaternion (body→world)
    // dt        : time since last call [s]
    //
    // Returns a WIOOutput. Check output.valid before using.
    // output.v_L and output.v_R are set even on invalid outputs (to 0.0).
    WIOOutput compute(double       pulses_L,
                      double       pulses_R,
                      const Quat&  q_imu,
                      double       dt);

    const WIOParams& getParams() const { return params_; }

private:
    WIOParams params_;

    // Previous encoder counts (for delta computation).
    double prev_pulses_L_ = 0.0;
    double prev_pulses_R_ = 0.0;
    bool   first_call_    = true;

    // Previous yaw for mid-point integration.
    double prev_yaw_ = 0.0;
};

} // namespace roar
