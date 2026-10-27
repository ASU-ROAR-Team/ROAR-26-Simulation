// ═════════════════════════════════════════════════════════════════════════════
// synthetic_imu_node.cpp
// ─────────────────────────────────────────────────────────────────────────────
// SIMULATION-ONLY node: fakes the BNO055 (NDOF mode) by numerically
// differentiating Gazebo ground truth.
//
// Subscribes:  /world/<world>/pose/info   (geometry_msgs/PoseArray)
// Publishes:   /imu/data                  (sensor_msgs/Imu)
//
// What a BNO055 in NDOF mode gives the filter:
//   - orientation           : absolute quaternion (fused)      → passthrough GT
//   - angular_velocity      : body-frame gyro                  → dq/dt
//   - linear_acceleration   : GRAVITY-REMOVED, body frame      → d²p/dt²
//
// Numerical differentiation of a ~58 Hz pose stream is noisy, so the
// acceleration is smoothed with an exponential moving average (ema_alpha).
// A short warmup (default 30 samples ≈ 0.5 s) suppresses the initial
// garbage before velocities settle — the filter side tolerates this because
// WIO's max_delta_pulses was sized for the post-warmup first step.
// ═════════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <Eigen/Dense>
#include <string>

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;

class SyntheticImu : public rclcpp::Node
{
public:
    SyntheticImu() : Node("synthetic_imu_node")
    {
        gt_topic_ = declare_parameter<std::string>(
            "ground_truth_topic", "/world/marsyard/pose/info");
        model_name_ = declare_parameter<std::string>("model_name", "roar_rover");
        pose_index_ = declare_parameter<int>("pose_index", 11);
        out_topic_  = declare_parameter<std::string>("output_topic", "/imu/data");
        ema_alpha_  = declare_parameter<double>("ema_alpha", 0.5);
        warmup_samples_ = declare_parameter<int>("warmup_samples", 30);

        // ── Robustness guards (added after marsyard RTF-stutter testing) ─────
        // Gazebo on slower machines runs at RTF 10-45% and delivers pose
        // messages in bursts: several samples microseconds apart, then a gap.
        // Differentiating those bursts with wall-clock dt produces absurd
        // velocities/accelerations that blow up the filter. Three guards:
        //   min_dt     : ignore samples closer than this — differentiation
        //                then spans a sane interval (burst collapses to one)
        //   max_speed  : a sample implying speed above this is garbage
        //                (rover can't exceed ~2 m/s) — resync and skip
        //   max_accel  : clamp the computed acceleration (rover physically
        //                cannot exceed a few m/s²)
        min_dt_    = declare_parameter<double>("min_dt",    0.005);   // [s]
        max_speed_ = declare_parameter<double>("max_speed", 5.0);     // [m/s]
        max_accel_ = declare_parameter<double>("max_accel", 20.0);    // [m/s²]

        sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            gt_topic_, 10,
            std::bind(&SyntheticImu::callback, this, std::placeholders::_1));
        pub_ = create_publisher<sensor_msgs::msg::Imu>(out_topic_, 10);

        RCLCPP_INFO(get_logger(), "Synthetic IMU started");
        RCLCPP_INFO(get_logger(), "  %s[%d] -> %s  (ema=%.2f warmup=%d)",
            gt_topic_.c_str(), pose_index_, out_topic_.c_str(),
            ema_alpha_, warmup_samples_);
    }

private:
    void callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (pose_index_ < 0 ||
            static_cast<size_t>(pose_index_) >= msg->poses.size()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "pose_index=%d out of range (%zu poses)",
                pose_index_, msg->poses.size());
            return;
        }

        const auto& pose = msg->poses[pose_index_];
        const double t = now().seconds();

        const Vec3 p(pose.position.x, pose.position.y, pose.position.z);
        Quat q(pose.orientation.w, pose.orientation.x,
               pose.orientation.y, pose.orientation.z);
        q.normalize();

        if (!have_prev_) {
            prev_p_ = p; prev_q_ = q; prev_t_ = t;
            have_prev_ = true;
            return;
        }

        const double dt = t - prev_t_;

        // ── GUARD 1: minimum dt ──────────────────────────────────────────────
        // Skip burst samples (do NOT update prev_* — the next accepted sample
        // then differentiates over a sane interval).
        if (dt < min_dt_) return;

        // ── Velocity (world frame) ────────────────────────────────────────────
        const Vec3 v_world = (p - prev_p_) / dt;

        // ── GUARD 2: speed sanity ────────────────────────────────────────────
        // A sample implying impossible speed is a timing glitch or teleport.
        // Resync (accept the pose as the new reference) but skip this cycle's
        // velocity/accel and restart accel differencing.
        if (v_world.norm() > max_speed_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Synthetic IMU: rejecting sample (implied speed %.1f m/s > %.1f)",
                v_world.norm(), max_speed_);
            prev_p_ = p; prev_q_ = q; prev_t_ = t;
            have_prev_v_ = false;
            return;
        }

        // ── Acceleration (world → body, gravity-free) ────────────────────────
        Vec3 a_world = Vec3::Zero();
        if (have_prev_v_) {
            a_world = (v_world - prev_v_world_) / dt;
        }
        const Eigen::Matrix3d R = q.toRotationMatrix();
        Vec3 a_body_raw = R.transpose() * a_world;

        // ── GUARD 3: acceleration clamp ──────────────────────────────────────
        const double a_norm = a_body_raw.norm();
        if (a_norm > max_accel_) {
            a_body_raw *= (max_accel_ / a_norm);
        }

        // EMA smoothing
        a_body_ema_ = ema_alpha_ * a_body_raw + (1.0 - ema_alpha_) * a_body_ema_;

        // ── Angular velocity (body frame) from quaternion difference ────────
        // dq = q_prev⁻¹ ⊗ q_curr  →  axis·angle / dt
        Quat dq = prev_q_.conjugate() * q;
        dq.normalize();
        if (dq.w() < 0.0) dq.coeffs() *= -1.0;   // shortest arc
        const Eigen::AngleAxisd aa(dq);
        const Vec3 omega_body = aa.axis() * aa.angle() / dt;

        // ── Save state ────────────────────────────────────────────────────────
        prev_p_ = p; prev_q_ = q; prev_t_ = t;
        prev_v_world_ = v_world;
        have_prev_v_ = true;

        // ── Warmup gate ───────────────────────────────────────────────────────
        if (sample_count_ < warmup_samples_) {
            sample_count_++;
            return;
        }

        // ── Publish ───────────────────────────────────────────────────────────
        sensor_msgs::msg::Imu imu;
        imu.header.stamp = now();
        imu.header.frame_id = "imu_link";

        imu.orientation.w = q.w();
        imu.orientation.x = q.x();
        imu.orientation.y = q.y();
        imu.orientation.z = q.z();
        imu.orientation_covariance[0] = 1e-6;
        imu.orientation_covariance[4] = 1e-6;
        imu.orientation_covariance[8] = 1e-6;

        imu.angular_velocity.x = omega_body.x();
        imu.angular_velocity.y = omega_body.y();
        imu.angular_velocity.z = omega_body.z();
        imu.angular_velocity_covariance[0] = 1e-5;
        imu.angular_velocity_covariance[4] = 1e-5;
        imu.angular_velocity_covariance[8] = 1e-5;

        imu.linear_acceleration.x = a_body_ema_.x();
        imu.linear_acceleration.y = a_body_ema_.y();
        imu.linear_acceleration.z = a_body_ema_.z();
        imu.linear_acceleration_covariance[0] = 1e-3;
        imu.linear_acceleration_covariance[4] = 1e-3;
        imu.linear_acceleration_covariance[8] = 1e-3;

        pub_->publish(imu);
    }

    std::string gt_topic_;
    std::string model_name_;
    std::string out_topic_;
    int pose_index_ = 11;
    double ema_alpha_ = 0.5;
    int warmup_samples_ = 30;
    double min_dt_    = 0.005;
    double max_speed_ = 5.0;
    double max_accel_ = 20.0;

    bool have_prev_   = false;
    bool have_prev_v_ = false;
    int  sample_count_ = 0;
    Vec3 prev_p_ = Vec3::Zero();
    Quat prev_q_ = Quat::Identity();
    double prev_t_ = 0.0;
    Vec3 prev_v_world_ = Vec3::Zero();
    Vec3 a_body_ema_   = Vec3::Zero();

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyntheticImu>());
    rclcpp::shutdown();
    return 0;
}
