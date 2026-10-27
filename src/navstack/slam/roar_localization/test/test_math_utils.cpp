//
// test_math_utils.cpp
//
// Tests for math_utils.hpp: skew, exp_so3, quat_to_rot, rot_to_euler,
// wrap_angle. These are inline-only utilities, so this test is header-only.
//

#include <iostream>
#include <cmath>
#include "roar_localization/math_utils.hpp"

using namespace roar;

static int passed = 0;
static int failed = 0;

#define CHECK(name, cond) \
    do { \
        if (cond) { std::cout << "  PASS  " << (name) << "\n"; ++passed; } \
        else       { std::cout << "  FAIL  " << (name) << "\n"; ++failed; } \
    } while(0)

// ─────────────────────────────────────────────────────────────────────────────
// 1. skew() produces correct skew-symmetric matrix and obeys v×w = skew(v)·w
// ─────────────────────────────────────────────────────────────────────────────
void test_skew()
{
    std::cout << "\n[1. skew()]\n";
    Vec3 v(1.0, 2.0, 3.0);
    Mat3 S = skew(v);

    CHECK("S is skew-symmetric", (S + S.transpose()).norm() < 1e-12);

    Vec3 w(4.0, -1.0, 2.0);
    Vec3 expected = v.cross(w);
    Vec3 actual   = S * w;
    CHECK("S·w == v×w", (expected - actual).norm() < 1e-12);
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. exp_so3 of zero is Identity
// ─────────────────────────────────────────────────────────────────────────────
void test_exp_so3_zero()
{
    std::cout << "\n[2. exp_so3(0) = I]\n";
    Mat3 R = exp_so3(Vec3::Zero());
    CHECK("identity", (R - Mat3::Identity()).norm() < 1e-12);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. exp_so3 of 90° about z-axis is the expected matrix
// ─────────────────────────────────────────────────────────────────────────────
void test_exp_so3_z90()
{
    std::cout << "\n[3. exp_so3(π/2 · ẑ) = R_z(90°)]\n";
    Vec3 phi(0.0, 0.0, M_PI / 2.0);
    Mat3 R = exp_so3(phi);

    Mat3 expected;
    expected <<  0.0, -1.0,  0.0,
                 1.0,  0.0,  0.0,
                 0.0,  0.0,  1.0;

    CHECK("matches R_z(90°)", (R - expected).norm() < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. exp_so3 produces a valid rotation matrix (orthogonal, det = 1)
// ─────────────────────────────────────────────────────────────────────────────
void test_exp_so3_orthogonal()
{
    std::cout << "\n[4. exp_so3 → orthogonal matrix]\n";
    Vec3 phi(0.7, -0.4, 1.2);
    Mat3 R = exp_so3(phi);

    Mat3 RRt = R * R.transpose();
    CHECK("R·Rᵀ ≈ I", (RRt - Mat3::Identity()).norm() < 1e-9);
    CHECK("det(R) ≈ 1", std::abs(R.determinant() - 1.0) < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. quat_to_rot of identity quaternion is Identity
// ─────────────────────────────────────────────────────────────────────────────
void test_quat_identity()
{
    std::cout << "\n[5. quat_to_rot(I) = I]\n";
    Quat q = Quat::Identity();
    Mat3 R = quat_to_rot(q);
    CHECK("identity", (R - Mat3::Identity()).norm() < 1e-12);
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. quat_to_rot then rot_to_quat is the identity transform
// ─────────────────────────────────────────────────────────────────────────────
void test_quat_round_trip()
{
    std::cout << "\n[6. quat ↔ rot round trip]\n";
    Quat q(Eigen::AngleAxisd(0.7, Vec3(0.3, 0.5, 0.8).normalized()));
    Mat3 R   = quat_to_rot(q);
    Quat q2  = rot_to_quat(R);

    // Quaternions are double-cover, so q and -q represent the same rotation
    const double d_plus  = (q.coeffs() - q2.coeffs()).norm();
    const double d_minus = (q.coeffs() + q2.coeffs()).norm();
    CHECK("round trip preserves rotation", std::min(d_plus, d_minus) < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. rot_to_euler extracts correct yaw from pure-yaw rotation
// ─────────────────────────────────────────────────────────────────────────────
void test_rot_to_euler_yaw()
{
    std::cout << "\n[7. rot_to_euler: pure yaw]\n";
    const double yaw_true = 0.5;   // ~28.6°
    Quat q(Eigen::AngleAxisd(yaw_true, Vec3::UnitZ()));
    Mat3 R = quat_to_rot(q);
    Vec3 rpy = rot_to_euler(R);

    CHECK("roll ≈ 0",  std::abs(rpy(0))             < 1e-9);
    CHECK("pitch ≈ 0", std::abs(rpy(1))             < 1e-9);
    CHECK("yaw correct", std::abs(rpy(2) - yaw_true) < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// 8. rot_to_euler extracts correct pitch (positive nose-down convention)
// ─────────────────────────────────────────────────────────────────────────────
void test_rot_to_euler_pitch()
{
    std::cout << "\n[8. rot_to_euler: pure pitch]\n";
    const double pitch_true = 0.3;
    Quat q(Eigen::AngleAxisd(pitch_true, Vec3::UnitY()));
    Mat3 R = quat_to_rot(q);
    Vec3 rpy = rot_to_euler(R);

    CHECK("roll ≈ 0",          std::abs(rpy(0))                   < 1e-9);
    CHECK("pitch correct",      std::abs(rpy(1) - pitch_true)      < 1e-9);
    CHECK("yaw ≈ 0",            std::abs(rpy(2))                   < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// 9. wrap_angle wraps various inputs into [−π, +π]
// ─────────────────────────────────────────────────────────────────────────────
void test_wrap_angle()
{
    std::cout << "\n[9. wrap_angle]\n";
    CHECK("0 stays 0",                std::abs(wrap_angle(0.0))               < 1e-12);
    CHECK("π/2 stays π/2",            std::abs(wrap_angle(M_PI/2) - M_PI/2)    < 1e-12);
    CHECK("3π/2 → -π/2",              std::abs(wrap_angle(3.0*M_PI/2) - (-M_PI/2)) < 1e-9);
    CHECK("-3π/2 → π/2",              std::abs(wrap_angle(-3.0*M_PI/2) - M_PI/2) < 1e-9);
    CHECK("5π → π",                   std::abs(wrap_angle(5.0*M_PI) - M_PI)    < 1e-9);
    CHECK("output always in [-π, π]",
          std::abs(wrap_angle(100.0)) <= M_PI + 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// 10. quat_to_euler round trip via Euler angle construction
// ─────────────────────────────────────────────────────────────────────────────
void test_quat_to_euler_round_trip()
{
    std::cout << "\n[10. quat_to_euler round trip]\n";
    const double roll_true  = 0.2;
    const double pitch_true = -0.15;
    const double yaw_true   = 0.8;

    // Build quaternion as ZYX extrinsic (yaw → pitch → roll)
    Quat q =
        Eigen::AngleAxisd(yaw_true,   Vec3::UnitZ()) *
        Eigen::AngleAxisd(pitch_true, Vec3::UnitY()) *
        Eigen::AngleAxisd(roll_true,  Vec3::UnitX());

    Vec3 rpy = quat_to_euler(q);
    CHECK("roll round trip",  std::abs(rpy(0) - roll_true)  < 1e-9);
    CHECK("pitch round trip", std::abs(rpy(1) - pitch_true) < 1e-9);
    CHECK("yaw round trip",   std::abs(rpy(2) - yaw_true)   < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    std::cout << "=== math_utils unit tests ===\n";

    test_skew();
    test_exp_so3_zero();
    test_exp_so3_z90();
    test_exp_so3_orthogonal();
    test_quat_identity();
    test_quat_round_trip();
    test_rot_to_euler_yaw();
    test_rot_to_euler_pitch();
    test_wrap_angle();
    test_quat_to_euler_round_trip();

    std::cout << "\n=== Results: "
              << passed << " passed, " << failed << " failed ===\n";

    return failed == 0 ? 0 : 1;
}
