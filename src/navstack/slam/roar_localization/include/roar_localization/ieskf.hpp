#pragma once

#include <Eigen/Dense>
#include "roar_localization/math_utils.hpp"

namespace roar {

// ─────────────────────────────────────────────────────────────────────────────
// IESKFState
// The nominal state vector. 12 components.
//
//   p   — position      [m]       world frame
//   v   — velocity      [m/s]     world frame
//   b_a — accel bias    [m/s²]    body frame
//   b_w — gyro  bias    [rad/s]   body frame
//
// Orientation is NOT in here. The BNO055 NDOF handles it. We read R directly
// from the quaternion every predict step.
// ─────────────────────────────────────────────────────────────────────────────
struct IESKFState
{
    Vec3 p   = Vec3::Zero();
    Vec3 v   = Vec3::Zero();
    Vec3 b_a = Vec3::Zero();
    Vec3 b_w = Vec3::Zero();

    Eigen::Matrix<double,12,1> toVector() const
    {
        Eigen::Matrix<double,12,1> x;
        x.segment<3>(0) = p;
        x.segment<3>(3) = v;
        x.segment<3>(6) = b_a;
        x.segment<3>(9) = b_w;
        return x;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// IESKFParams
// Noise parameters. Set once at startup, tune after rosbag tests.
//
// CONVENTION: sigma_an, sigma_aw, sigma_ww are NOISE DENSITIES [units/√Hz].
// Discretisation uses σ²·dt (matches Huang et al. 2024 + standard literature).
// ─────────────────────────────────────────────────────────────────────────────
struct IESKFParams
{
    // ── IMU noise densities ──────────────────────────────────────────────────
    double sigma_an = 3.0e-3;    // [m/s²/√Hz] accelerometer white noise
    double sigma_aw = 1.0e-4;    // [m/s²·√Hz] accel bias random walk
    double sigma_ww = 8.7e-5;    // [rad/s·√Hz] gyro bias random walk

    // ── Position observation noise (raw ZED VO) ──────────────────────────────
    double sigma_pos = 0.05;     // [m] standard deviation

    // ── ArUco observation noise ──────────────────────────────────────────────
    // ArUco is much more accurate than raw VO when the marker is close.
    // The actual noise used per detection is scaled by distance² (further =
    // less accurate, because pixel error projects to larger world error).
    //
    //   V_aruco = (sigma_aruco_base + sigma_aruco_per_m × distance)² × I₃
    //
    // Tune these so a 5 m detection has ~1-3 cm uncertainty and a 15 m
    // detection has ~5-15 cm uncertainty.
    double sigma_aruco_base  = 0.01;   // [m]     baseline noise (close range)
    double sigma_aruco_per_m = 0.005;  // [m/m]   noise growth per meter of distance

    // ── IESKF iteration control ──────────────────────────────────────────────
    int    max_iterations = 10;
    double epsilon        = 1e-6;

    // ── Physical sanity clamp ────────────────────────────────────────────────
    // The rover cannot physically exceed ~2 m/s. If the predicted velocity
    // norm exceeds max_velocity, it is clamped — this bounds the damage from
    // a single garbage IMU sample (sensor glitch, sim stutter) which would
    // otherwise send the position estimate flying until observations drag
    // it back. Set to the true vehicle limit with generous margin.
    double max_velocity   = 5.0;   // [m/s]
};

// ─────────────────────────────────────────────────────────────────────────────
// IESKF — Iterated Error-State Kalman Filter
//
// Observation types supported:
//   1. update(z_pos, k_V)
//        Position observation from WVO (ZED2i + WO blend). Default path,
//        called at ~30 Hz from the ZED callback.
//
//   2. updateVelocity(v_obs, sigma_v)
//        Velocity observation. Used for ZUPT (Zero Velocity Updates) when
//        the rover is detected as stationary: v_obs = [0,0,0].
//
//   3. updateAruco(id_position_world, p_marker_camera,
//                  R_cam_to_world, t_cam_in_world, sigma_aruco)
//        ArUco landmark observation. Given the marker's known world position
//        and its observed position in the camera frame, computes the implied
//        rover position and applies it as a high-confidence absolute fix.
//
// All three share the same Joseph-form covariance machinery and the
// iterated update structure.
// ─────────────────────────────────────────────────────────────────────────────
class IESKF
{
public:
    using StateVec   = Eigen::Matrix<double,12,1>;
    using StateMat   = Eigen::Matrix<double,12,12>;
    using ObsVec     = Eigen::Vector3d;
    using ObsMat     = Eigen::Matrix<double,3,12>;
    using ObsCov     = Eigen::Matrix3d;
    using KalmanGain = Eigen::Matrix<double,12,3>;

    explicit IESKF(const IESKFParams& params = IESKFParams{});

    // ── Initialisation ────────────────────────────────────────────────────────
    void setState(const IESKFState& state);
    void setCovariance(const StateMat& P);

    // ── PREDICT (~100 Hz) ─────────────────────────────────────────────────────
    //
    // k_Q  : WO-FIS scaling factor for Q_v (default 1.0 = no scaling)
    //         k_Q > 1 → inflate velocity process noise (wheel slip detected)
    void predict(const Vec3& a_m,
                 const Vec3& omega_m,
                 const Mat3& R,
                 double      vx,
                 const Vec3& dp_world,
                 double      dt,
                 double      k_Q = 1.0);

    // ── UPDATE: position (from WVO / ZED2i, ~30 Hz) ──────────────────────────
    //
    // z    : position measurement [m] world frame
    // k_V  : VO-FIS scaling factor for V (default 1.0)
    //         k_V > 1 → inflate observation noise (poor ZED tracking)
    void update(const ObsVec& z, double k_V = 1.0);

    // ── UPDATE: velocity (ZUPT) ───────────────────────────────────────────────
    //
    // v_obs   : observed velocity [m/s] world frame. For ZUPT this is [0,0,0].
    // sigma_v : measurement std dev [m/s]. Small (~0.01) for high confidence.
    //
    // Observation model:
    //   z = v + noise
    //   H = [0₃ | I₃ | 0₃ | 0₃]
    //
    // This collapses accumulated velocity drift AND indirectly corrects b_a
    // (the Kalman gain spreads the correction across the state correlations).
    void updateVelocity(const Vec3& v_obs, double sigma_v);

    // ── UPDATE: ArUco landmark observation ────────────────────────────────────
    //
    // The perception team gives us a marker detection in the camera frame.
    // We already know the marker's world position. Solving for the rover's
    // world position:
    //
    //   p_marker_world = p_rover_world + R_body_to_world · (
    //                       t_cam_in_body + R_cam_to_body · p_marker_camera )
    //
    //   ⇒ p_rover_world = p_marker_world − R_body_to_world ·
    //                         (t_cam_in_body + R_cam_to_body · p_marker_camera)
    //
    // The right-hand side is our position observation z_aruco.
    //
    // p_marker_world  : known marker position [m] world frame (from map)
    // p_marker_camera : measured marker position [m] in camera optical frame
    // R_body_to_world : rover body→world rotation (from BNO055)
    // t_cam_in_body   : translation cam→base_link in body frame [m]
    // R_cam_to_body   : rotation camera→body (fixed extrinsic)
    // sigma_aruco     : measurement std dev [m] (scaled by distance externally)
    void updateAruco(const Vec3& p_marker_world,
                     const Vec3& p_marker_camera,
                     const Mat3& R_body_to_world,
                     const Vec3& t_cam_in_body,
                     const Mat3& R_cam_to_body,
                     double      sigma_aruco);

    // ── Accessors ─────────────────────────────────────────────────────────────
    const IESKFState&  getState()      const { return state_; }
    const StateMat&    getCovariance() const { return P_; }
    const IESKFParams& getParams()     const { return params_; }

private:
    // Build the 12×12 state-transition Jacobian Fx.
    StateMat buildFx(const Mat3& R, double dt) const;

    // Build the 12×12 covariance noise contribution Fi·Qi·Fi'.
    // k_Q scales only the velocity noise block (slip inflates Q_v only).
    StateMat buildFiQiFi(double dt, double k_Q = 1.0) const;

    // Build H for position observation: [I₃ | 0 | 0 | 0]
    ObsMat buildH_position() const;

    // Build H for velocity observation: [0 | I₃ | 0 | 0]
    ObsMat buildH_velocity() const;

    // V = sigma² × I₃ — used by all three update types
    ObsCov makeV(double sigma) const;

    // Inject the converged error state into the nominal state.
    void injectErrorState(const StateVec& dx);

    // Generic update kernel — defined inline below the class because it is
    // a function template and must be visible at every call site.
    //
    //   z          : 3D observation
    //   H          : 3×12 observation Jacobian
    //   V          : 3×3 observation noise covariance
    //   predict_h  : nullary functor returning h(x) — the predicted obs
    //                at the current nominal state
    //   add_dx_to_h: binary functor (h_at_x, dx) → h(x ⊕ δx)
    template<typename PredictFn, typename DxFn>
    void applyUpdate(const ObsVec& z,
                     const ObsMat& H,
                     const ObsCov& V,
                     PredictFn     predict_h,
                     DxFn          add_dx_to_h)
    {
        // Innovation covariance S = H·P·H' + V  [3×3]
        const Eigen::Matrix3d S = H * P_ * H.transpose() + V;

        // Kalman gain K = P·H'·S⁻¹  [12×3]  (LDLT for symmetric PD solve)
        const KalmanGain K =
            P_ * H.transpose() * S.ldlt().solve(Eigen::Matrix3d::Identity());

        // Iterated update loop — standard IEKF (Bell & Cathey) form:
        //
        //   δx_{i+1} = K · ( z − h(x ⊕ δx_i) + H·δx_i )
        //
        // (algebraically equivalent to δx_{i+1} = K·(z − h(x⊕δx_i)) − (I−K·H)·δx_i)
        //
        // The +H·δx_i term is essential: it re-expresses the residual around
        // the ORIGINAL linearization point. For linear h this collapses to
        // z − h(x) and the loop converges in exactly one iteration with the
        // full Kalman step. For future nonlinear observations, the loop
        // performs proper Gauss-Newton iterations.
        StateVec dx_iter = StateVec::Zero();
        const ObsVec h_at_x = predict_h();

        for (int i = 0; i < params_.max_iterations; ++i)
        {
            const ObsVec h_x_dx   = add_dx_to_h(h_at_x, dx_iter);
            const ObsVec residual = z - h_x_dx + H * dx_iter;
            const StateVec dx_new = K * residual;

            const bool converged = (dx_new - dx_iter).norm() < params_.epsilon;
            dx_iter = dx_new;
            if (converged) break;
        }

        injectErrorState(dx_iter);

        // Joseph-form covariance update — keeps P symmetric positive-definite
        const StateMat I_KH = StateMat::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * V * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());
    }

    IESKFParams params_;
    IESKFState  state_;
    StateMat    P_;
};

} // namespace roar
