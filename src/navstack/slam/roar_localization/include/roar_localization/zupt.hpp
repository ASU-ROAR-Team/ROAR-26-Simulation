#pragma once

// ═════════════════════════════════════════════════════════════════════════════
// zupt.hpp
// ─────────────────────────────────────────────────────────────────────────────
// ZUPT — Zero Velocity Updates
//
// CORE IDEA (recap):
//   When the rover is physically stationary, its true velocity is exactly zero.
//   We can inject this as a free measurement into the IESKF:
//
//       z_zupt = [0, 0, 0]
//       H_zupt = [0₃ | I₃ | 0₃ | 0₃]   (observes velocity directly)
//
//   This collapses any accumulated velocity drift (and indirectly corrects
//   accelerometer bias b_a, because the only way velocity can grow during
//   the predict step in our 12D filter is via R·δb_a·dt — so the filter
//   attributes "velocity error → bias error" automatically).
//
// WHY NEEDED:
//   Even at rest, the accelerometer outputs `a_m = b_a + noise`. Integrating
//   this gives a slowly drifting velocity estimate (`v += R·(a_m - b_a)·dt`).
//   On rough terrain b_a varies with temperature and vibration, so the filter
//   never perfectly cancels it. ZUPT is the cheapest, most powerful way to
//   reset that drift whenever the rover stops — which happens naturally at
//   every waypoint, obstacle assessment, ArUco scan, planner re-evaluation.
//
// SAFETY: WHY MULTI-CONDITION GATING MATTERS
//   We must NEVER fire ZUPT if the rover is actually moving. False ZUPT
//   triggers force the filter to believe velocity is zero when it isn't,
//   producing position drift. The classic failure mode is:
//     - rover stuck in soft sand, wheels spinning at command speed
//     - encoders report velocity ≠ 0 (good — they correctly say "moving")
//     - IMU reports near-zero accel (rover is actually stuck, not moving)
//   We use encoder velocity as the primary check, not IMU acceleration alone.
//
// CONDITIONS FOR STATIONARY:
//   ALL of the following must be true for `min_stationary_duration` seconds:
//     1. |v_L| < encoder_speed_threshold
//     2. |v_R| < encoder_speed_threshold
//     3. |ω_z| < angular_velocity_threshold       (from BNO055)
//     4. accel_variance < accel_variance_threshold (over a ring buffer)
//
//   Holding all four conditions for a sustained period (e.g., 0.5 s) avoids
//   false triggers from momentary lulls (e.g., a brief pause between strides
//   while traversing rough terrain).
//
// USAGE:
//   ZuptDetector detector(params);
//   detector.update(v_L, v_R, omega_z, a_m, now);
//   if (detector.isStationary()) {
//       filter.updateVelocity(Vec3::Zero(), V_zupt);
//   }
// ═════════════════════════════════════════════════════════════════════════════

#include <deque>
#include <cmath>
#include "roar_localization/math_utils.hpp"

namespace roar {

// ─────────────────────────────────────────────────────────────────────────────
// ZuptParams
// All thresholds tunable from launch params.
// ─────────────────────────────────────────────────────────────────────────────
struct ZuptParams
{
    // Encoder wheel speed below which we consider the wheel "not spinning".
    // Default 0.02 m/s = 2 cm/s — easily below realistic crawl speed.
    double encoder_speed_threshold = 0.02;     // [m/s]

    // Yaw rate below which we consider the rover "not rotating".
    // BNO055 NDOF has typical noise ~0.01 rad/s when stationary, so 0.05 rad/s
    // gives ~5× margin.
    double angular_velocity_threshold = 0.05;  // [rad/s]

    // Acceleration variance (across the buffer) below which the IMU sees
    // a "quiet" rover. Computed as the trace of the sample covariance of
    // the last `accel_buffer_size` accel readings.
    double accel_variance_threshold = 0.05;    // [m²/s⁴]

    // Number of recent acceleration samples used for variance estimation.
    // At 100 Hz IMU rate, 30 samples ≈ 0.3 s of history.
    int    accel_buffer_size = 30;

    // The rover must satisfy all four conditions for at least this many
    // seconds continuously before we declare "stationary". Prevents
    // false triggers from momentary lulls during normal driving.
    double min_stationary_duration = 0.5;      // [s]

    // Observation noise for the ZUPT measurement [m/s]. Very small =
    // very confident that velocity is exactly zero. Set slightly larger
    // than zero to keep the filter numerically well-conditioned.
    double sigma_v_zupt = 0.01;                // [m/s]
};

// ─────────────────────────────────────────────────────────────────────────────
// ZuptDetector
//
// State machine that tracks whether the rover is currently stationary based
// on a sliding window of encoder, gyro, and accel readings.
// ─────────────────────────────────────────────────────────────────────────────
class ZuptDetector
{
public:
    explicit ZuptDetector(const ZuptParams& params = ZuptParams{})
        : params_(params) {}

    // Reset internal buffers (call on startup or after a long pause).
    void reset()
    {
        accel_buffer_.clear();
        first_satisfied_time_ = 0.0;
        satisfied_now_        = false;
        currently_stationary_ = false;
    }

    // Feed new sensor readings. Call at IMU rate (~100 Hz).
    //
    //   v_L, v_R   : wheel surface speeds from WIO [m/s]
    //   omega_z    : yaw rate from BNO055 (body frame) [rad/s]
    //   a_m        : raw IMU acceleration (body frame, gravity-removed) [m/s²]
    //   now_sec    : current time [s] (use the filter's clock so dt/sim/wall
    //                domains agree)
    void update(double v_L, double v_R,
                double omega_z,
                const Vec3& a_m,
                double now_sec)
    {
        // ── Update accel ring buffer ─────────────────────────────────────────
        accel_buffer_.push_back(a_m);
        while (static_cast<int>(accel_buffer_.size()) > params_.accel_buffer_size) {
            accel_buffer_.pop_front();
        }

        // ── Evaluate all four conditions ─────────────────────────────────────
        const bool wheels_quiet =
            std::abs(v_L) < params_.encoder_speed_threshold &&
            std::abs(v_R) < params_.encoder_speed_threshold;

        const bool gyro_quiet =
            std::abs(omega_z) < params_.angular_velocity_threshold;

        const bool accel_quiet =
            (static_cast<int>(accel_buffer_.size()) >= params_.accel_buffer_size) &&
            (computeAccelVariance() < params_.accel_variance_threshold);

        const bool all_satisfied = wheels_quiet && gyro_quiet && accel_quiet;

        // ── Update timer ─────────────────────────────────────────────────────
        // If all conditions just became true, remember when. If they remain
        // true for `min_stationary_duration`, declare stationary.
        if (all_satisfied) {
            if (!satisfied_now_) {
                first_satisfied_time_ = now_sec;
                satisfied_now_        = true;
            }
            const double elapsed = now_sec - first_satisfied_time_;
            currently_stationary_ = (elapsed >= params_.min_stationary_duration);
        } else {
            satisfied_now_        = false;
            currently_stationary_ = false;
        }
    }

    // Is the rover currently considered stationary?
    bool isStationary() const { return currently_stationary_; }

    // Diagnostic accessors.
    bool wheelsQuiet(double v_L, double v_R) const
    {
        return std::abs(v_L) < params_.encoder_speed_threshold &&
               std::abs(v_R) < params_.encoder_speed_threshold;
    }

    double currentAccelVariance() const
    {
        return computeAccelVariance();
    }

    const ZuptParams& getParams() const { return params_; }

private:
    // Compute the trace of the sample covariance of the accel buffer.
    // We use the trace (sum of variances along x/y/z) as a single scalar
    // "noise" indicator. Returns 0 if the buffer is empty.
    double computeAccelVariance() const
    {
        if (accel_buffer_.empty()) return 0.0;

        // Mean
        Vec3 mean = Vec3::Zero();
        for (const Vec3& a : accel_buffer_) mean += a;
        mean /= static_cast<double>(accel_buffer_.size());

        // Trace of sample covariance
        double var_trace = 0.0;
        for (const Vec3& a : accel_buffer_) {
            const Vec3 d = a - mean;
            var_trace += d.squaredNorm();
        }
        var_trace /= static_cast<double>(accel_buffer_.size());
        return var_trace;
    }

    ZuptParams         params_;
    std::deque<Vec3>   accel_buffer_;
    double             first_satisfied_time_ = 0.0;
    bool               satisfied_now_        = false;
    bool               currently_stationary_ = false;
};

} // namespace roar
