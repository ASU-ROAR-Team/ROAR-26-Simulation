// ═════════════════════════════════════════════════════════════════════════════
// aruco_simulator_node.cpp
// ─────────────────────────────────────────────────────────────────────────────
// SIMULATION-ONLY node that fakes the perception team's ArUco detector.
//
// HOW IT WORKS:
//   1. Loads the same ArUco map YAML as the IESKF node — the markers' known
//      world positions.
//   2. Subscribes to the rover's ground-truth pose from Gazebo
//      (PoseArray on /world/<world>/pose/info, same source as
//      ground_truth_bridge in the existing simulation).
//   3. For every camera frame (publishes at a fixed rate, ~10 Hz to match
//      typical ArUco detection rates), checks each marker against:
//        - distance limit (max_distance)
//        - field-of-view limit (horizontal_fov / vertical_fov)
//        - line-of-sight (currently always true; no terrain occlusion model)
//   4. For markers that pass all checks, computes the marker's position in
//      the camera optical frame, adds Gaussian noise, and publishes an
//      ArucoDetectionArray.
//
// WHY THIS EXISTS:
//   To validate the IESKF's ArUco update path before the perception team's
//   real detector is ready. The interface (ArucoDetectionArray) is identical
//   to what perception will eventually publish.
//
// CAMERA MODEL:
//   Same conventions as the IESKF node:
//     - Camera optical frame: x=right, y=down, z=forward
//     - Camera mounted at t_cam_in_body in rover body frame
//     - Body→camera rotation is the inverse of the IESKF node's R_cam_to_body
//   The FOV check is approximate (cone from optical axis); for sim purposes
//   this is plenty.
// ═════════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <roar_msgs/msg/aruco_detection_array.hpp>

#include "roar_localization/math_utils.hpp"
#include "roar_localization/aruco_map.hpp"

#include <random>
#include <cmath>
#include <string>

using namespace roar;
using std::placeholders::_1;

class ArucoSimulator : public rclcpp::Node
{
public:
    ArucoSimulator() : Node("aruco_simulator")
    {
        // ── Parameters ────────────────────────────────────────────────────────
        gt_topic_ = declare_parameter<std::string>(
            "ground_truth_topic", "/world/marsyard/pose/info");
        out_topic_ = declare_parameter<std::string>(
            "aruco_topic", "/aruco/detections");
        model_name_ = declare_parameter<std::string>("model_name", "roar_rover");
        pose_index_ = declare_parameter<int>("pose_index", 11);

        const std::string aruco_map_path =
            declare_parameter<std::string>("aruco_map_path", "");
        if (aruco_map_path.empty()) {
            RCLCPP_ERROR(get_logger(),
                "aruco_map_path is empty — simulator will publish no detections.");
        } else {
            try {
                aruco_map_.loadFromYaml(aruco_map_path);
                RCLCPP_INFO(get_logger(),
                    "Loaded %zu ArUco markers from %s",
                    aruco_map_.size(), aruco_map_path.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(),
                    "Failed to load ArUco map: %s", e.what());
            }
        }

        // Camera extrinsic (same conventions as IESKF node)
        t_cam_in_body_ = Vec3(
            declare_parameter<double>("cam_to_base_x", 0.0),
            declare_parameter<double>("cam_to_base_y", 0.0),
            declare_parameter<double>("cam_to_base_z", 0.3));

        // ZED optical-to-body standard
        Mat3 R_optical_to_body;
        R_optical_to_body <<  0.0,  0.0,  1.0,
                             -1.0,  0.0,  0.0,
                              0.0, -1.0,  0.0;
        const double d2r = M_PI / 180.0;
        const double cam_roll_deg  = declare_parameter<double>("cam_roll_deg",  0.0);
        const double cam_pitch_deg = declare_parameter<double>("cam_pitch_deg", 0.0);
        const double cam_yaw_deg   = declare_parameter<double>("cam_yaw_deg",   0.0);
        const Quat q_extra =
            Eigen::AngleAxisd(cam_yaw_deg   * d2r, Vec3::UnitZ()) *
            Eigen::AngleAxisd(cam_pitch_deg * d2r, Vec3::UnitY()) *
            Eigen::AngleAxisd(cam_roll_deg  * d2r, Vec3::UnitX());
        R_cam_to_body_ = q_extra.toRotationMatrix() * R_optical_to_body;
        R_body_to_cam_ = R_cam_to_body_.transpose();

        // Camera FOV
        const double hfov_deg = declare_parameter<double>("horizontal_fov_deg", 90.0);
        const double vfov_deg = declare_parameter<double>("vertical_fov_deg",   60.0);
        cos_h_half_fov_ = std::cos(hfov_deg * 0.5 * d2r);
        cos_v_half_fov_ = std::cos(vfov_deg * 0.5 * d2r);

        // Range
        max_distance_ = declare_parameter<double>("max_distance",     15.0);
        min_distance_ = declare_parameter<double>("min_distance",      0.3);

        // Noise
        noise_base_      = declare_parameter<double>("noise_base",      0.005);
        noise_per_meter_ = declare_parameter<double>("noise_per_meter", 0.003);
        noise_seed_      = declare_parameter<int>   ("noise_seed",      42);
        rng_.seed(noise_seed_);

        // Detection rate
        const double rate = declare_parameter<double>("detection_rate_hz", 10.0);
        const auto period = std::chrono::duration<double>(1.0 / rate);

        // Sticky last-known rover pose
        rover_pose_valid_ = false;

        // ── ROS interfaces ────────────────────────────────────────────────────
        sub_gt_ = create_subscription<geometry_msgs::msg::PoseArray>(
            gt_topic_, 10,
            std::bind(&ArucoSimulator::groundTruthCallback, this, _1));

        pub_aruco_ = create_publisher<roar_msgs::msg::ArucoDetectionArray>(
            out_topic_, 10);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&ArucoSimulator::timerCallback, this));

        RCLCPP_INFO(get_logger(),
            "ArucoSimulator: gt=%s out=%s rate=%.1fHz fov=%.0fx%.0f deg "
            "range=[%.1f, %.1f] m noise=[%.3f + %.3f×d] m",
            gt_topic_.c_str(), out_topic_.c_str(), rate,
            hfov_deg, vfov_deg, min_distance_, max_distance_,
            noise_base_, noise_per_meter_);
    }

private:
    void groundTruthCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (pose_index_ < 0 ||
            static_cast<size_t>(pose_index_) >= msg->poses.size()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "pose_index=%d out of range (got %zu poses)",
                pose_index_, msg->poses.size());
            return;
        }
        const auto& pose = msg->poses[pose_index_];

        rover_pos_ = Vec3(pose.position.x,
                          pose.position.y,
                          pose.position.z);
        rover_quat_ = Quat(pose.orientation.w,
                           pose.orientation.x,
                           pose.orientation.y,
                           pose.orientation.z);
        rover_quat_.normalize();
        rover_pose_valid_ = true;
    }

    void timerCallback()
    {
        if (!rover_pose_valid_) return;
        if (aruco_map_.size() == 0) return;

        const Mat3 R_body_to_world = quat_to_rot(rover_quat_);
        const Mat3 R_world_to_body = R_body_to_world.transpose();

        roar_msgs::msg::ArucoDetectionArray out_msg;
        out_msg.header.stamp    = now();
        out_msg.header.frame_id = "camera_optical";

        // Iterate every marker and check visibility
        for (int id : aruco_map_.ids())
        {
            const Vec3 p_marker_world = *aruco_map_.getMarkerPosition(id);

            // Marker in body frame
            const Vec3 p_marker_in_body =
                R_world_to_body * (p_marker_world - rover_pos_);

            // Marker in camera optical frame
            // p_cam = R_body_to_cam · (p_body − t_cam_in_body)
            const Vec3 p_marker_camera =
                R_body_to_cam_ * (p_marker_in_body - t_cam_in_body_);

            // Reject if behind camera (z <= 0 in optical frame)
            if (p_marker_camera.z() <= 0.0) continue;

            // Distance check
            const double d = p_marker_camera.norm();
            if (d < min_distance_ || d > max_distance_) continue;

            // FOV checks (approximate cone)
            // Horizontal: cos(angle from optical axis in x-z plane)
            const double cos_h = p_marker_camera.z() /
                std::sqrt(p_marker_camera.x()*p_marker_camera.x() +
                          p_marker_camera.z()*p_marker_camera.z() + 1e-12);
            if (cos_h < cos_h_half_fov_) continue;

            // Vertical: cos(angle from optical axis in y-z plane)
            const double cos_v = p_marker_camera.z() /
                std::sqrt(p_marker_camera.y()*p_marker_camera.y() +
                          p_marker_camera.z()*p_marker_camera.z() + 1e-12);
            if (cos_v < cos_v_half_fov_) continue;

            // Add Gaussian noise (distance-scaled)
            const double sigma = noise_base_ + noise_per_meter_ * d;
            std::normal_distribution<double> noise(0.0, sigma);
            const Vec3 p_noisy(
                p_marker_camera.x() + noise(rng_),
                p_marker_camera.y() + noise(rng_),
                p_marker_camera.z() + noise(rng_));

            // Build the detection
            roar_msgs::msg::ArucoDetection det;
            det.header = out_msg.header;
            det.id     = id;
            det.pose_in_camera.position.x = p_noisy.x();
            det.pose_in_camera.position.y = p_noisy.y();
            det.pose_in_camera.position.z = p_noisy.z();
            // Orientation: identity (we don't simulate marker orientation;
            // IESKF ignores it anyway, only position is used)
            det.pose_in_camera.orientation.w = 1.0;
            det.pose_in_camera.orientation.x = 0.0;
            det.pose_in_camera.orientation.y = 0.0;
            det.pose_in_camera.orientation.z = 0.0;
            det.distance = static_cast<float>(p_noisy.norm());

            out_msg.detections.push_back(det);
        }

        // Publish even if empty — downstream can use timestamps
        pub_aruco_->publish(out_msg);
    }

    // Parameters / state
    std::string gt_topic_;
    std::string out_topic_;
    std::string model_name_;
    int    pose_index_     = 11;
    double max_distance_   = 15.0;
    double min_distance_   = 0.3;
    double noise_base_     = 0.005;
    double noise_per_meter_= 0.003;
    int    noise_seed_     = 42;
    double cos_h_half_fov_ = 0.0;
    double cos_v_half_fov_ = 0.0;

    ArucoMap aruco_map_;

    Vec3 t_cam_in_body_;
    Mat3 R_cam_to_body_;
    Mat3 R_body_to_cam_;

    // Latest ground truth
    Vec3 rover_pos_ = Vec3::Zero();
    Quat rover_quat_= Quat::Identity();
    bool rover_pose_valid_ = false;

    std::mt19937 rng_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_gt_;
    rclcpp::Publisher<roar_msgs::msg::ArucoDetectionArray>::SharedPtr pub_aruco_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoSimulator>());
    rclcpp::shutdown();
    return 0;
}
