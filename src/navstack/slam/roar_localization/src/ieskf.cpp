#include "roar_localization/ieskf.hpp"
#include <stdexcept>
#include <cmath>

namespace roar {

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
IESKF::IESKF(const IESKFParams& params)
    : params_(params)
    , P_(StateMat::Identity() * 0.01)
{
}

void IESKF::setState(const IESKFState& state)  { state_ = state; }
void IESKF::setCovariance(const StateMat& P)   { P_ = P; }

// ─────────────────────────────────────────────────────────────────────────────
// buildFx — 12×12 state-transition Jacobian
//
// Block layout (each 3×3):
//        δp       δv       δb_a     δb_w
//   δp [ I₃     I₃·dt      0        0   ]
//   δv [ 0       I₃       −R·dt     0   ]
//  δb_a[ 0       0         I₃       0   ]
//  δb_w[ 0       0         0        I₃  ]
//
// Why −R·dt at (3,6):
//   b_a is in the body frame. The corrected accel a_c = a_m − b_a is rotated
//   to world by R, so a body-frame bias error δb_a causes a world-frame
//   velocity error of −R·δb_a·dt over one step.
// ─────────────────────────────────────────────────────────────────────────────
IESKF::StateMat IESKF::buildFx(const Mat3& R, double dt) const
{
    StateMat Fx = StateMat::Identity();
    Fx.block<3,3>(0, 3) = Mat3::Identity() * dt;   // δp += δv·dt
    Fx.block<3,3>(3, 6) = -R * dt;                 // δv += −R·δb_a·dt
    return Fx;
}

// ─────────────────────────────────────────────────────────────────────────────
// buildFiQiFi — 12×12 process noise contribution
//
// Q_v  = σ_an² · dt · I₃ · k_Q   (k_Q from WO-FIS, default 1.0)
// Q_ba = σ_aw² · dt · I₃         (NOT scaled — bias walk is IMU-intrinsic)
// Q_bw = σ_ww² · dt · I₃         (NOT scaled)
//
// NOTE: dt (not dt²). sigma_an is a noise density [m/s²/√Hz], so the
// discrete-time velocity variance over one step is σ²·dt.
// ─────────────────────────────────────────────────────────────────────────────
IESKF::StateMat IESKF::buildFiQiFi(double dt, double k_Q) const
{
    StateMat FiQiFi = StateMat::Zero();

    const double san2 = params_.sigma_an * params_.sigma_an;
    const double saw2 = params_.sigma_aw * params_.sigma_aw;
    const double sww2 = params_.sigma_ww * params_.sigma_ww;

    FiQiFi.block<3,3>(3, 3) = Mat3::Identity() * san2 * dt * k_Q;   // Q_v
    FiQiFi.block<3,3>(6, 6) = Mat3::Identity() * saw2 * dt;          // Q_ba
    FiQiFi.block<3,3>(9, 9) = Mat3::Identity() * sww2 * dt;          // Q_bw

    return FiQiFi;
}

// ─────────────────────────────────────────────────────────────────────────────
// Observation matrices
// ─────────────────────────────────────────────────────────────────────────────
IESKF::ObsMat IESKF::buildH_position() const
{
    ObsMat H = ObsMat::Zero();
    H.block<3,3>(0, 0) = Mat3::Identity();   // selects δp
    return H;
}

IESKF::ObsMat IESKF::buildH_velocity() const
{
    ObsMat H = ObsMat::Zero();
    H.block<3,3>(0, 3) = Mat3::Identity();   // selects δv
    return H;
}

IESKF::ObsCov IESKF::makeV(double sigma) const
{
    return ObsCov::Identity() * (sigma * sigma);
}

// ─────────────────────────────────────────────────────────────────────────────
// injectErrorState
// Adds the converged error state δx into the nominal state x.
// ─────────────────────────────────────────────────────────────────────────────
void IESKF::injectErrorState(const StateVec& dx)
{
    state_.p   += dx.segment<3>(0);
    state_.v   += dx.segment<3>(3);
    state_.b_a += dx.segment<3>(6);
    state_.b_w += dx.segment<3>(9);
    // Orientation NOT injected — BNO055 owns it.
}

// ─────────────────────────────────────────────────────────────────────────────
// predict — runs at ~100 Hz
// ─────────────────────────────────────────────────────────────────────────────
void IESKF::predict(const Vec3& a_m,
                    const Vec3& /*omega_m*/,
                    const Mat3& R,
                    double      /*vx*/,
                    const Vec3& dp_world,
                    double      dt,
                    double      k_Q)
{
    if (dt <= 0.0) return;

    const Vec3 a_c = a_m - state_.b_a;

    // Nominal state propagation (exact, nonlinear)
    state_.p += dp_world;          // position via WIO dead-reckoning
    state_.v += R * a_c * dt;      // velocity via corrected accel (world frame)

    // ── Physical sanity clamp ─────────────────────────────────────────────
    // Bound the velocity to what the vehicle can physically do. A single
    // garbage accel sample can otherwise blow the velocity up and send the
    // position estimate flying (observed under Gazebo RTF stutter; the same
    // guard protects the real rover from IMU glitches).
    const double v_norm = state_.v.norm();
    if (v_norm > params_.max_velocity) {
        state_.v *= (params_.max_velocity / v_norm);
    }

    // b_a, b_w stay constant (random walk — only update step modifies them)
    // Orientation: not propagated (BNO055 owns it)

    // Error-state covariance propagation
    const StateMat Fx     = buildFx(R, dt);
    const StateMat FiQiFi = buildFiQiFi(dt, k_Q);

    P_ = Fx * P_ * Fx.transpose() + FiQiFi;
    P_ = 0.5 * (P_ + P_.transpose());   // enforce symmetry
}

// ─────────────────────────────────────────────────────────────────────────────
// update — position observation (WVO / ZED2i)
// ─────────────────────────────────────────────────────────────────────────────
void IESKF::update(const ObsVec& z, double k_V)
{
    const ObsMat H = buildH_position();
    const ObsCov V = makeV(params_.sigma_pos * std::sqrt(k_V));
    // Equivalently: V = sigma_pos² × k_V × I₃ (matches the original formula).

    applyUpdate(z, H, V,
        // predict_h: current predicted observation = state.p
        [&]() -> ObsVec { return state_.p; },
        // add_dx_to_h: h(x ⊕ δx) = (p + δp). Linear obs.
        [](const ObsVec& h, const StateVec& dx) -> ObsVec {
            return h + dx.segment<3>(0);
        }
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// updateVelocity — ZUPT or any other velocity observation
// ─────────────────────────────────────────────────────────────────────────────
void IESKF::updateVelocity(const Vec3& v_obs, double sigma_v)
{
    const ObsMat H = buildH_velocity();
    const ObsCov V = makeV(sigma_v);

    applyUpdate(v_obs, H, V,
        // predict_h: current predicted observation = state.v
        [&]() -> ObsVec { return state_.v; },
        // add_dx_to_h: h(x ⊕ δx) = (v + δv)
        [](const ObsVec& h, const StateVec& dx) -> ObsVec {
            return h + dx.segment<3>(3);
        }
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// updateAruco — landmark observation from known marker world position
//
// Geometry derivation:
//
//   The camera sees the marker at p_marker_camera in its optical frame.
//   We know the camera is at offset (t_cam_in_body) from base_link, expressed
//   in body frame. We know the marker's world position p_marker_world.
//
//   In the world frame:
//     p_marker_world = p_rover_world
//                    + R_body_to_world × (t_cam_in_body
//                                         + R_cam_to_body × p_marker_camera)
//
//   Rearranging for the rover position:
//     p_rover_world = p_marker_world
//                   − R_body_to_world × (t_cam_in_body
//                                        + R_cam_to_body × p_marker_camera)
//
//   This is our z — a direct position observation, same H matrix as the
//   regular position update, just with a much tighter covariance.
//
// Noise scaling:
//   The actual measurement noise grows with marker distance. ArUco corner
//   detection in pixels translates to world-frame uncertainty that scales
//   roughly linearly with depth. The caller provides the final sigma_aruco
//   (after applying the distance scaling).
// ─────────────────────────────────────────────────────────────────────────────
void IESKF::updateAruco(const Vec3& p_marker_world,
                        const Vec3& p_marker_camera,
                        const Mat3& R_body_to_world,
                        const Vec3& t_cam_in_body,
                        const Mat3& R_cam_to_body,
                        double      sigma_aruco)
{
    // Implied rover world position from this marker detection
    const Vec3 marker_in_body  = t_cam_in_body + R_cam_to_body * p_marker_camera;
    const Vec3 marker_in_world = R_body_to_world * marker_in_body;
    const Vec3 z_aruco         = p_marker_world - marker_in_world;

    // Re-use the regular position observation path with the tighter sigma.
    const ObsMat H = buildH_position();
    const ObsCov V = makeV(sigma_aruco);

    applyUpdate(z_aruco, H, V,
        [&]() -> ObsVec { return state_.p; },
        [](const ObsVec& h, const StateVec& dx) -> ObsVec {
            return h + dx.segment<3>(0);
        }
    );
}

} // namespace roar
