// ═════════════════════════════════════════════════════════════════════════════
// ground_truth_bridge.cpp — SIMULATION ONLY
// ─────────────────────────────────────────────────────────────────────────────
// Reads Gazebo world pose as tf2_msgs/TFMessage (roar_rover_noise's
// basic_rover.launch.py bridges this on /world/marsyard/pose/info — the
// world file it loads, marsyard2024.world, names itself "marsyard"
// internally, which is what Ignition uses for the pose topic path) and
// republishes the rover's pose as a perfect fake ZED2i topic that IESKF fuses.
//
// This matches the ORIGINAL architecture where the "ZED" IS ground truth
// (giving sub-cm error). No noise, no drift, no bias — just direct pass-through.
// ═════════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <string>

class GroundTruthBridge : public rclcpp::Node
{
public:
    GroundTruthBridge() : Node("ground_truth_bridge")
    {
        gt_topic_ = declare_parameter<std::string>(
            "ground_truth_topic", "/world/marsyard/pose/info");
        rover_frame_ = declare_parameter<std::string>(
            "rover_frame", "roar_rover");
        out_topic_  = declare_parameter<std::string>(
            "output_topic", "/zed2i/zed_node/pose_with_covariance");
        position_var_ = declare_parameter<double>(
            "position_variance", 1.0e-6);

        sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
            gt_topic_, 10,
            std::bind(&GroundTruthBridge::callback, this,
                      std::placeholders::_1));
        pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            out_topic_, 10);

        RCLCPP_INFO(get_logger(),
            "Ground Truth Bridge started (perfect ZED simulation)");
        RCLCPP_INFO(get_logger(),
            "  input : %s  (TFMessage, child_frame_id='%s')",
            gt_topic_.c_str(), rover_frame_.c_str());
        RCLCPP_INFO(get_logger(),
            "  output: %s",
            out_topic_.c_str());
    }

private:
    void callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto& t : msg->transforms) {
            if (t.child_frame_id == rover_frame_) {
                geometry_msgs::msg::PoseWithCovarianceStamped out;
                out.header.stamp = now();
                out.header.frame_id = "odom";
                out.pose.pose.position.x = t.transform.translation.x;
                out.pose.pose.position.y = t.transform.translation.y;
                out.pose.pose.position.z = t.transform.translation.z;
                out.pose.pose.orientation = t.transform.rotation;

                for (auto& c : out.pose.covariance) c = 0.0;
                out.pose.covariance[0]  = position_var_;
                out.pose.covariance[7]  = position_var_;
                out.pose.covariance[14] = position_var_;
                out.pose.covariance[21] = 1e-6;
                out.pose.covariance[28] = 1e-6;
                out.pose.covariance[35] = 1e-6;

                pub_->publish(out);
                return;
            }
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Rover frame '%s' not found in TFMessage (%zu transforms)",
            rover_frame_.c_str(), msg->transforms.size());
    }

    std::string gt_topic_;
    std::string rover_frame_;
    std::string out_topic_;
    double position_var_;

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundTruthBridge>());
    rclcpp::shutdown();
    return 0;
}
