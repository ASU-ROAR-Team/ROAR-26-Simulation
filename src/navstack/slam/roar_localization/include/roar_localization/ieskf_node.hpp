#pragma once

#include <rclcpp/rclcpp.hpp>

// ROS2 message types
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

// Custom messages
#include <roar_msgs/msg/aruco_detection_array.hpp>

// Filter modules
#include "roar_localization/math_utils.hpp"
#include "roar_localization/ieskf.hpp"
#include "roar_localization/wio.hpp"
#include "roar_localization/wvo.hpp"
#include "roar_localization/fuzzy_utils.hpp"
#include "roar_localization/zupt.hpp"
#include "roar_localization/aruco_map.hpp"

namespace roar {

// ─────────────────────────────────────────────────────────────────────────────
// IESKFNode
//
// Top-level ROS2 node wiring all subsystems together.
//
// Subscriptions:
//   /imu/data                            — BNO055 IMU (~100 Hz)
//   /joint_states                        — Wheel encoders (~50 Hz)
//   /zed2i/zed_node/pose_with_covariance — ZED visual odometry (~30 Hz)
//   /aruco/detections                    — ArUco marker detections (event-driven)
//
// Publications:
//   /roar/odom_ieskf                     — fused odometry (~100 Hz)
//   TF: odom → base_link                 — coordinate transform (~100 Hz)
//   TF: map  → odom (static, once)       — identity, roots /map in the TF
//                                           tree; no IESKF state
//
// Internal pipelines:
//   IMU callback   → WIO → WO-FIS → IESKF.predict → publish
//                  → ZUPT detector update
//                    └── if stationary: IESKF.updateVelocity(0)
//
//   Joint callback → store encoder counts (consumed by IMU callback)
//
//   ZED callback   → ZED extrinsic correction
//                  → WVO → VO-FIS → IESKF.update
//
//   ArUco callback → for each detection:
//                      lookup marker world position
//                      IESKF.updateAruco
// ─────────────────────────────────────────────────────────────────────────────
class IESKFNode : public rclcpp::Node
{
public:
    explicit IESKFNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

private:
    // ── Callbacks ─────────────────────────────────────────────────────────────
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void zedPoseCallback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void arucoCallback(
        const roar_msgs::msg::ArucoDetectionArray::SharedPtr msg);

    // ── Publishing helpers ────────────────────────────────────────────────────
    void publishOdometry(const rclcpp::Time& stamp);
    void publishTF(const rclcpp::Time& stamp);
    void publishStaticMapToOdomTF();

    // ── Parameter loading ─────────────────────────────────────────────────────
    void loadParams();

    // ── Subsystems ────────────────────────────────────────────────────────────
    std::unique_ptr<IESKF>        filter_;
    std::unique_ptr<WIO>          wio_;
    std::unique_ptr<WVO>          wvo_;
    fuzzy::WOFIS                  wo_fis_;
    fuzzy::VOFIS                  vo_fis_;
    std::unique_ptr<ZuptDetector> zupt_;
    ArucoMap                      aruco_map_;

    // ── IMU mounting correction ──────────────────────────────────────────────
    // R_imu_to_body_ converts IMU-frame vectors to body frame.
    // Built from imu_roll_deg, imu_pitch_deg, imu_yaw_deg launch params.
    Mat3 R_imu_to_body_;

    // ── ZED2i extrinsic ──────────────────────────────────────────────────────
    // t_cam_in_body_  : ZED optical center position in body frame
    // R_cam_to_body_  : rotation from camera optical frame to body frame
    //
    // Standard ZED optical convention: x=right, y=down, z=forward.
    // Standard rover body convention:  x=forward, y=left, z=up.
    // So R_cam_to_body_ default is the standard optical-to-body permutation,
    // adjustable by extra mounting rotation if the camera is tilted.
    Vec3 t_cam_in_body_;
    Mat3 R_cam_to_body_;

    // ── Encoder state ────────────────────────────────────────────────────────
    double encoder_left_  = 0.0;
    double encoder_right_ = 0.0;
    bool   encoder_ready_ = false;
    std::string joint_left_;
    std::string joint_right_;

    // ── WVO dead-reckoning accumulator ───────────────────────────────────────
    Vec3 accumulated_dp_wo_ = Vec3::Zero();

    // ── WIO failure tracking ─────────────────────────────────────────────────
    int wio_fail_count_               = 0;
    static constexpr int kMaxWioFails = 10;   // ~100 ms at 100 Hz

    // ── FIS normalisation ────────────────────────────────────────────────────
    double max_omega_z_   = 1.5;   // [rad/s]
    double max_vx_        = 2.0;   // [m/s]
    double zed_cov_scale_ = 0.01;  // [m²]

    // ── ArUco settings ───────────────────────────────────────────────────────
    // Max detection distance — ignore markers further than this (per-detection
    // noise grows linearly with distance, so very distant detections are
    // borderline useless and just inject noise).
    double aruco_max_distance_     = 15.0;   // [m]
    // Detected markers we will accept — set empty to accept all known IDs.
    // (e.g., restrict to certain IDs if perception team uses some IDs for
    // other purposes.)
    bool   aruco_enabled_          = true;

    // ── ZUPT settings ────────────────────────────────────────────────────────
    bool   zupt_enabled_           = true;

    // ── Timing ───────────────────────────────────────────────────────────────
    rclcpp::Time last_imu_time_;
    bool         first_imu_ = true;

    // Latest BNO055 quaternion (body→world after mounting correction).
    Quat current_orientation_ = Quat::Identity();

    // ── ROS interface ────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr               sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr        sub_joints_;
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr        sub_zed_;
    rclcpp::Subscription<roar_msgs::msg::ArucoDetectionArray>::SharedPtr sub_aruco_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                pub_odom_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr       pub_diag_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                       tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster>                 static_tf_broadcaster_;

    // ── Diagnostics counters (consumed by slam_evaluator) ────────────────────
    // Published on /roar/ieskf_diagnostics as Float64MultiArray:
    //   [0] aruco detections received (total, incl. skipped)
    //   [1] aruco corrections applied (total)
    //   [2] zupt updates fired (total)
    //   [3] position covariance trace  [m²]
    //   [4] last k_Q from WO-FIS
    //   [5] last k_V from VO-FIS
    uint64_t diag_aruco_received_ = 0;
    uint64_t diag_aruco_applied_  = 0;
    uint64_t diag_zupt_fired_     = 0;
    double   diag_last_k_Q_       = 1.0;
    double   diag_last_k_V_       = 1.0;
    int      diag_decimator_      = 0;

    bool        publish_tf_ = true;
    std::string frame_id_;
    std::string child_frame_id_;
    std::string odom_topic_;
    std::string aruco_topic_;

    // ── map → odom bridging ──────────────────────────────────────────────────
    // This node only estimates odom → base_footprint. It has no separate global
    // localization source, so map → odom is broadcast once as a fixed identity
    // transform on /tf_static purely to give /map a place at the root of the
    // TF tree (e.g. for mapping nodes that publish/lookup in the map frame).
    // It carries no IESKF state and does not affect the filter in any way.
    //
    // Published as a STATIC transform (not re-sent on /tf every cycle) so it
    // stays valid regardless of clock domain: tf2 treats /tf_static entries as
    // valid for any query time, sidestepping any possible mismatch between this
    // node's use_sim_time clock and whatever stamps odom → base_footprint
    // immediately below it (this node, when publish_tf_ is true; previously
    // Gazebo's diff_drive plugin in launches where publish_tf_ was false).
    bool        publish_map_to_odom_ = true;
    std::string map_frame_id_;

    // NOTE: there used to be an output-only `output_yaw_offset_deg_` here that
    // rotated the *reported* orientation to paper over a 90deg body-frame
    // mismatch, without correcting current_orientation_ used for fusion. The
    // actual mismatch is that base_link (where the IMU is mounted) is yawed
    // -90deg from the rover's true forward (base_footprint) — see
    // base_footprint_joint in roar_simulation's urdf/base/rover.xacro. That is
    // now corrected at the input via `imu_yaw_deg` (R_imu_to_body_, set in the
    // launch files), which fixes the orientation used for fusion itself, so
    // no output-side patch is needed.
};

} // namespace roar
