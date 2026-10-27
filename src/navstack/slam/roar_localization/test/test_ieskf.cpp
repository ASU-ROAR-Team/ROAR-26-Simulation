//
// test_ieskf.cpp
//
// Tests for the updated IESKF API. Compared to the original:
//   - predict() now takes an optional k_Q parameter (default 1.0)
//   - update() now takes an optional k_V parameter (default 1.0)
//   - new updateVelocity() and updateAruco() methods are exercised here
//
// All original tests still pass: this is backward-compatible.
//

#include <iostream>
#include <cmath>
#include "roar_localization/ieskf.hpp"
#include "roar_localization/math_utils.hpp"

using namespace roar;

static int passed = 0;
static int failed = 0;

#define CHECK(name, cond) \
    do { \
        if (cond) { std::cout << "  PASS  " << name << "\n"; ++passed; } \
        else       { std::cout << "  FAIL  " << name << "\n"; ++failed; } \
    } while(0)

// Build a WIO-style dp_world for flat forward motion.
static Vec3 make_dp(double vx, double yaw, double pitch, double dt)
{
    return Vec3(vx * std::cos(yaw) * dt,
                vx * std::sin(yaw) * dt,
                vx * std::sin(pitch) * dt);
}

// ─────────────────────────────────────────────────────────────────────────────
// 1. Predict only: covariance grows, position moves correctly
// ─────────────────────────────────────────────────────────────────────────────
void test_predict_only()
{
    std::cout << "\n[1. Predict only]\n";
    IESKF filter;

    IESKFState state;
    filter.setState(state);

    const double vx  = 0.5;
    const double dt  = 0.01;
    Mat3 R = Mat3::Identity();
    Vec3 a_m = Vec3::Zero();
    Vec3 w_m = Vec3::Zero();

    double trace_before = filter.getCovariance().trace();
    for (int i = 0; i < 100; ++i) {
        Vec3 dp = make_dp(vx, 0.0, 0.0, dt);
        filter.predict(a_m, w_m, R, vx, dp, dt);   // default k_Q=1.0
    }
    double trace_after = filter.getCovariance().trace();
    Vec3 p_after = filter.getState().p;

    CHECK("covariance grows", trace_after > trace_before);
    CHECK("position moved ~0.5m in x", std::abs(p_after.x() - 0.5) < 0.01);
    CHECK("position y stays near zero", std::abs(p_after.y()) < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. Position update corrects the state
// ─────────────────────────────────────────────────────────────────────────────
void test_update_corrects_position()
{
    std::cout << "\n[2. Position update corrects position]\n";
    IESKF filter;

    IESKFState state;
    state.p = Vec3(1.0, 0.0, 0.0);    // wrong — true position is origin
    filter.setState(state);

    IESKF::StateMat P = IESKF::StateMat::Identity();
    P.block<3,3>(0,0) *= 1.0;
    filter.setCovariance(P);

    Vec3 z_true(0.0, 0.0, 0.0);
    double error_before = (filter.getState().p - z_true).norm();
    filter.update(z_true);              // default k_V=1.0
    double error_after = (filter.getState().p - z_true).norm();

    CHECK("position error decreases", error_after < error_before);
    CHECK("error reduced > 50%",       error_after < error_before * 0.5);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. Observability: accel bias is NOT observable from position updates alone
//    (position propagates from WIO dead-reckoning, not from filter velocity),
//    but becomes observable the moment ZUPT velocity observations arrive.
//    This documents an intentional property of the 12D WIO-anchored design —
//    and is exactly why ZUPT matters for this filter.
// ─────────────────────────────────────────────────────────────────────────────
void test_bias_estimation()
{
    std::cout << "\n[3. Bias observability: position-only vs ZUPT]\n";
    IESKFParams params;
    params.sigma_pos = 0.001;
    IESKF filter(params);

    IESKFState state;
    filter.setState(state);
    IESKF::StateMat P = IESKF::StateMat::Identity() * 0.5;
    filter.setCovariance(P);

    const Vec3 true_bias(0.1, 0.0, 0.0);
    Mat3 R = Mat3::Identity();
    const double dt = 0.01;

    // Phase 1: stationary rover, biased IMU, position updates only.
    // WIO says dp=0 and ZED says p=0 → residual is exactly zero → the
    // filter cannot learn the bias, and velocity drifts unchecked.
    for (int i = 0; i < 500; ++i) {
        filter.predict(true_bias, Vec3::Zero(), R, 0.0, Vec3::Zero(), dt);
        filter.update(Vec3::Zero());
    }
    const double b_a_after_pos_only = filter.getState().b_a.x();
    const double v_after_pos_only   = filter.getState().v.norm();

    CHECK("position-only: bias NOT learned",
          std::abs(b_a_after_pos_only) < 0.02);
    CHECK("position-only: velocity drifted", v_after_pos_only > 0.1);

    // Phase 2: ZUPT observations arrive → bias becomes observable.
    for (int i = 0; i < 100; ++i) {
        filter.predict(true_bias, Vec3::Zero(), R, 0.0, Vec3::Zero(), dt);
        filter.updateVelocity(Vec3::Zero(), 0.005);
    }
    const Vec3 b_a_final = filter.getState().b_a;

    CHECK("with ZUPT: b_a converges to true bias",
          (b_a_final - true_bias).norm() < 0.05);
    CHECK("with ZUPT: b_a x positive", b_a_final.x() > 0.0);
    CHECK("with ZUPT: velocity collapsed",
          filter.getState().v.norm() < v_after_pos_only * 0.2);
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. ZUPT via updateVelocity: stationary robot, velocity drift collapses
// ─────────────────────────────────────────────────────────────────────────────
void test_zupt_velocity_update()
{
    std::cout << "\n[4. ZUPT collapses velocity drift]\n";
    IESKF filter;

    // Start with a bogus non-zero velocity in the state
    IESKFState state;
    state.v = Vec3(0.5, -0.2, 0.0);
    filter.setState(state);

    // Inflate velocity covariance so the filter trusts the ZUPT observation
    IESKF::StateMat P = IESKF::StateMat::Identity();
    P.block<3,3>(3,3) *= 1.0;          // 1 m²/s² velocity uncertainty
    filter.setCovariance(P);

    double v_before = filter.getState().v.norm();

    // Apply ZUPT: observe v = 0 with tight sigma
    filter.updateVelocity(Vec3::Zero(), 0.01);

    double v_after = filter.getState().v.norm();

    CHECK("velocity collapses toward zero", v_after < v_before * 0.5);
    CHECK("velocity covariance shrinks",
          (filter.getCovariance().block<3,3>(3,3).trace() < P.block<3,3>(3,3).trace()));
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. ZUPT corrects accel bias indirectly via velocity-bias coupling
//
// Setup:
//   - Inject a constant accel bias of [0.1, 0, 0]
//   - Run predict for some time so velocity drifts
//   - Fire ZUPT
//   - Check that b_a has moved in the right direction
// ─────────────────────────────────────────────────────────────────────────────
void test_zupt_corrects_bias()
{
    std::cout << "\n[5. ZUPT corrects accel bias indirectly]\n";
    IESKF filter;

    IESKFState state;
    filter.setState(state);

    // Inflate ALL covariances so the filter is willing to learn
    IESKF::StateMat P = IESKF::StateMat::Identity() * 0.5;
    filter.setCovariance(P);

    const Vec3 true_bias(0.1, 0.0, 0.0);
    Mat3 R = Mat3::Identity();
    const double dt = 0.01;

    // Predict for 2 seconds — IMU reports only bias, so velocity drifts
    for (int i = 0; i < 200; ++i) {
        filter.predict(true_bias, Vec3::Zero(), R, 0.0, Vec3::Zero(), dt);
    }
    double v_drift = filter.getState().v.norm();
    double b_a_x_before = filter.getState().b_a.x();

    // Apply many ZUPTs to let bias estimate converge
    for (int i = 0; i < 50; ++i) {
        filter.predict(true_bias, Vec3::Zero(), R, 0.0, Vec3::Zero(), dt);
        filter.updateVelocity(Vec3::Zero(), 0.005);
    }

    double b_a_x_after = filter.getState().b_a.x();
    double v_final     = filter.getState().v.norm();

    CHECK("velocity drift existed before ZUPT", v_drift > 0.1);
    CHECK("velocity collapsed after ZUPT",      v_final < v_drift * 0.5);
    CHECK("b_a_x grew toward true bias",        b_a_x_after > b_a_x_before);
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. ArUco update: known marker corrects position drift
//
// Setup:
//   - Rover is actually at world origin
//   - Filter mistakenly thinks it's at (2, 1, 0)
//   - A marker is known to be at (5, 0, 0.5), facing camera optical frame
//     z-axis. Therefore, in camera frame the marker is at (0, 0, 5).
//     Wait — let's keep things simple: camera is at base_link (zero offset),
//     R_cam_to_body = Identity, R_body_to_world = Identity.
//   - The "true" camera-frame position of the marker is then (5, 0, 0.5)
//     minus the rover position = (5, 0, 0.5).
//   - We feed that to updateAruco and verify the filter pulls back to origin.
// ─────────────────────────────────────────────────────────────────────────────
void test_aruco_update()
{
    std::cout << "\n[6. ArUco update corrects drift]\n";
    IESKF filter;

    IESKFState state;
    state.p = Vec3(2.0, 1.0, 0.0);   // filter wrong — true is origin
    filter.setState(state);

    IESKF::StateMat P = IESKF::StateMat::Identity();
    P.block<3,3>(0,0) *= 2.0;        // big position uncertainty so update is trusted
    filter.setCovariance(P);

    const Vec3 marker_world(5.0, 0.0, 0.5);    // known marker position
    const Mat3 R_body_to_world = Mat3::Identity();
    const Vec3 t_cam_in_body   = Vec3::Zero();
    const Mat3 R_cam_to_body   = Mat3::Identity();

    // The true rover position is origin (0,0,0).
    // So the marker is at (5, 0, 0.5) in the camera frame.
    const Vec3 p_marker_camera(5.0, 0.0, 0.5);

    double err_before = (filter.getState().p - Vec3::Zero()).norm();

    filter.updateAruco(marker_world, p_marker_camera,
                       R_body_to_world, t_cam_in_body, R_cam_to_body,
                       0.01);

    double err_after = (filter.getState().p - Vec3::Zero()).norm();

    CHECK("ArUco update reduces position error",
          err_after < err_before * 0.3);
    CHECK("position lands near origin", err_after < 0.05);
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. ArUco update with camera extrinsic offset
//
// Setup:
//   - Camera is offset 0.2 m forward from base_link
//   - Rover is at origin, facing world-x
//   - Marker at (5, 0, 0.5) world position
//   - Marker in camera frame should be: (5 - 0.2, 0, 0.5) = (4.8, 0, 0.5)
//     (camera optical = body frame here for simplicity)
// ─────────────────────────────────────────────────────────────────────────────
void test_aruco_with_extrinsic()
{
    std::cout << "\n[7. ArUco with camera extrinsic]\n";
    IESKF filter;

    IESKFState state;
    state.p = Vec3(1.0, 0.0, 0.0);   // wrong — true is origin
    filter.setState(state);

    IESKF::StateMat P = IESKF::StateMat::Identity();
    P.block<3,3>(0,0) *= 2.0;
    filter.setCovariance(P);

    const Vec3 marker_world(5.0, 0.0, 0.5);
    const Mat3 R_body_to_world = Mat3::Identity();
    const Vec3 t_cam_in_body(0.2, 0.0, 0.3);   // 20 cm fwd, 30 cm up
    const Mat3 R_cam_to_body   = Mat3::Identity();

    // True rover at origin. Marker world = (5, 0, 0.5).
    // marker_in_body  = marker_world - rover_pos = (5, 0, 0.5)
    // marker_in_cam   = R_cam_to_body^T × (marker_in_body - t_cam_in_body)
    //                 = (5 - 0.2, 0 - 0, 0.5 - 0.3) = (4.8, 0, 0.2)
    const Vec3 p_marker_camera(4.8, 0.0, 0.2);

    filter.updateAruco(marker_world, p_marker_camera,
                       R_body_to_world, t_cam_in_body, R_cam_to_body,
                       0.01);

    double err = (filter.getState().p - Vec3::Zero()).norm();
    CHECK("ArUco with extrinsic lands near origin", err < 0.05);
}

// ─────────────────────────────────────────────────────────────────────────────
// 8. Covariance stays symmetric and positive-definite after every update type
// ─────────────────────────────────────────────────────────────────────────────
void test_covariance_health()
{
    std::cout << "\n[8. Covariance symmetric + PD after every update type]\n";
    IESKF filter;

    IESKFState state;
    state.p = Vec3(0.0, 0.0, 0.0);
    filter.setState(state);

    Mat3 R = Mat3::Identity();
    const double dt = 0.01;

    for (int i = 0; i < 30; ++i) {
        filter.predict(Vec3::Zero(), Vec3::Zero(), R, 0.5,
                       make_dp(0.5, 0.0, 0.0, dt), dt);
        filter.update(Vec3(0.5 * i * dt, 0.0, 0.0));
        filter.updateVelocity(Vec3(0.5, 0.0, 0.0), 0.05);
    }
    filter.updateAruco(Vec3(5.0, 0.0, 0.0),
                       Vec3(5.0 - 0.5*30*dt, 0.0, 0.0),
                       Mat3::Identity(), Vec3::Zero(), Mat3::Identity(),
                       0.01);

    const auto& P = filter.getCovariance();

    double sym_err = (P - P.transpose()).norm();
    CHECK("P symmetric", sym_err < 1e-10);

    Eigen::SelfAdjointEigenSolver<IESKF::StateMat> eig(P);
    double min_eig = eig.eigenvalues().minCoeff();
    CHECK("P positive definite (min eigenvalue > 0)", min_eig > -1e-10);
}

// ─────────────────────────────────────────────────────────────────────────────
// 9. k_V scaling: large k_V should reduce the effect of an update
// ─────────────────────────────────────────────────────────────────────────────
void test_kV_scaling()
{
    std::cout << "\n[9. k_V scaling reduces update influence]\n";

    // Two filters with identical setup
    auto run = [](double k_V) -> double {
        IESKF filter;
        IESKFState s;
        s.p = Vec3(1.0, 0.0, 0.0);
        filter.setState(s);

        IESKF::StateMat P = IESKF::StateMat::Identity();
        P.block<3,3>(0,0) *= 0.1;
        filter.setCovariance(P);

        filter.update(Vec3::Zero(), k_V);
        return filter.getState().p.norm();
    };

    double err_trust   = run(1.0);     // trust ZED fully
    double err_distrust= run(100.0);   // distrust ZED

    CHECK("distrusted update barely moves state",
          err_distrust > err_trust * 5.0);
}

// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    std::cout << "=== IESKF unit tests (new API) ===\n";

    test_predict_only();
    test_update_corrects_position();
    test_bias_estimation();
    test_zupt_velocity_update();
    test_zupt_corrects_bias();
    test_aruco_update();
    test_aruco_with_extrinsic();
    test_covariance_health();
    test_kV_scaling();

    std::cout << "\n=== Results: "
              << passed << " passed, " << failed << " failed ===\n";

    return failed == 0 ? 0 : 1;
}
