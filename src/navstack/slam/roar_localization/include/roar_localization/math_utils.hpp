#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace roar {

// ─────────────────────────────────────────────────────────────────────────────
// Type aliases
// ─────────────────────────────────────────────────────────────────────────────
using Vec3  = Eigen::Vector3d;
using Vec4  = Eigen::Vector4d;
using Mat3  = Eigen::Matrix3d;
using Mat4  = Eigen::Matrix4d;
using Quat  = Eigen::Quaterniond;

// ─────────────────────────────────────────────────────────────────────────────
// skew(v)
// Returns the 3×3 skew-symmetric (cross-product) matrix of a 3-vector.
//
//   skew([a, b, c]) = [  0  -c   b ]
//                     [  c   0  -a ]
//                     [ -b   a   0 ]
//
// Used in: Fx Jacobian (−R·skew(a_c)·dt block when orientation is in state),
//          SO(3) exponential map.
// ─────────────────────────────────────────────────────────────────────────────
inline Mat3 skew(const Vec3& v)
{
    Mat3 S;
    S <<     0.0, -v(2),  v(1),
           v(2),   0.0, -v(0),
          -v(1),  v(0),   0.0;
    return S;
}

// ─────────────────────────────────────────────────────────────────────────────
// exp_so3(phi)
// Rodrigues rotation: maps a rotation vector φ ∈ ℝ³ to a rotation matrix R ∈ SO(3).
//
//   R = I + sin(θ)/θ · K + (1 - cos(θ))/θ² · K²
//   where K = skew(φ/θ),  θ = ‖φ‖
//
// For small θ (< 1e-8) falls back to first-order approximation R ≈ I + skew(φ)
// to avoid division by zero.
//
// Used in: integrating gyroscope angular velocity (not needed in our 12D state
// since BNO055 owns orientation, but kept for WIO yaw integration and testing).
// ─────────────────────────────────────────────────────────────────────────────
inline Mat3 exp_so3(const Vec3& phi)
{
    const double theta = phi.norm();
    if (theta < 1e-8) {
        return Mat3::Identity() + skew(phi);
    }
    const Vec3 axis = phi / theta;
    const double s  = std::sin(theta);
    const double c  = std::cos(theta);
    return c * Mat3::Identity()
         + (1.0 - c) * axis * axis.transpose()
         + s * skew(axis);
}

// ─────────────────────────────────────────────────────────────────────────────
// quat_to_rot(q)
// Converts a unit quaternion to a 3×3 rotation matrix.
// q = [qx, qy, qz, qw]  (Eigen storage order: x, y, z, w)
//
// The BNO055 NDOF outputs a quaternion every step.
// We call this once per predict step to get R (body → world).
// ─────────────────────────────────────────────────────────────────────────────
inline Mat3 quat_to_rot(const Quat& q)
{
    return q.normalized().toRotationMatrix();
}

// ─────────────────────────────────────────────────────────────────────────────
// rot_to_quat(R)
// Converts a rotation matrix to a unit quaternion.
// ─────────────────────────────────────────────────────────────────────────────
inline Quat rot_to_quat(const Mat3& R)
{
    return Quat(R).normalized();
}

// ─────────────────────────────────────────────────────────────────────────────
// rot_to_euler(R)  →  [roll, pitch, yaw]  in radians
//
// Convention: ZYX extrinsic  (yaw first, then pitch, then roll).
// This matches the standard ROS / aerospace convention.
//
// roll  = atan2( R(2,1), R(2,2) )
// pitch = asin ( -R(2,0) )          [clamped to ±1 to avoid NaN]
// yaw   = atan2( R(1,0), R(0,0) )
//
// Used in: WIO step 2 to extract yaw for dead reckoning.
// ─────────────────────────────────────────────────────────────────────────────
inline Vec3 rot_to_euler(const Mat3& R)
{
    const double roll  = std::atan2( R(2,1), R(2,2) );
    const double pitch = std::asin ( std::max(-1.0, std::min(1.0, -R(2,0))) );
    const double yaw   = std::atan2( R(1,0), R(0,0) );
    return Vec3(roll, pitch, yaw);
}

// ─────────────────────────────────────────────────────────────────────────────
// wrap_angle(angle)
// Wraps any angle to [−π, +π].
// Used in WIO yaw differencing to avoid ±180° jumps.
// ─────────────────────────────────────────────────────────────────────────────
inline double wrap_angle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ─────────────────────────────────────────────────────────────────────────────
// quat_to_euler(q)  →  [roll, pitch, yaw]
// Convenience wrapper: quaternion → rotation matrix → Euler.
// ─────────────────────────────────────────────────────────────────────────────
inline Vec3 quat_to_euler(const Quat& q)
{
    return rot_to_euler(quat_to_rot(q));
}

// ─────────────────────────────────────────────────────────────────────────────
// rot_to_rpy_rates(R, omega_body)
// Converts body-frame angular velocity ω_body [rad/s] to Euler-angle rates
// [ṙoll, ṗitch, ẏaw]. Useful for debugging — not used inside the filter.
//
// The Jacobian T maps body angular velocity to Euler rates:
//   [ṙ, ṗ, ẏ]ᵀ = T(roll, pitch) · ω_body
// ─────────────────────────────────────────────────────────────────────────────
inline Vec3 rot_to_rpy_rates(const Mat3& R, const Vec3& omega_body)
{
    const Vec3 euler = rot_to_euler(R);
    const double roll  = euler(0);
    const double pitch = euler(1);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);

    // Avoid singularity at pitch = ±90°
    if (std::abs(cp) < 1e-6) {
        return Vec3::Zero();
    }

    Eigen::Matrix3d T;
    T << 1.0,  sr*sp/cp,  cr*sp/cp,
         0.0,  cr,        -sr,
         0.0,  sr/cp,      cr/cp;

    return T * omega_body;
}

} // namespace roar
