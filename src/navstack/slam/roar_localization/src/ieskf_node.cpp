#include "roar_localization/ieskf_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace roar {

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
IESKFNode::IESKFNode(const rclcpp::NodeOptions& options)
    : Node("ieskf_node", options)
{
    loadParams();

    // ── Subscribers ───────────────────────────────────────────────────────────
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&IESKFNode::imuCallback, this, std::placeholders::_1));

    sub_joints_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&IESKFNode::jointStateCallback, this, std::placeholders::_1));

    sub_zed_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/zed2i/zed_node/pose_with_covariance", 10,
        std::bind(&IESKFNode::zedPoseCallback, this, std::placeholders::_1));

    sub_aruco_ = this->create_subscription<roar_msgs::msg::ArucoDetectionArray>(
        aruco_topic_, 10,
        std::bind(&IESKFNode::arucoCallback, this, std::placeholders::_1));

    // ── Publishers ────────────────────────────────────────────────────────────
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic_, 10);
    pub_diag_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/roar/ieskf_diagnostics", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    if (publish_map_to_odom_) publishStaticMapToOdomTF();

    // ── Status banner ─────────────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "IESKF node started.");
    RCLCPP_INFO(get_logger(), "  IMU      : /imu/data");
    RCLCPP_INFO(get_logger(), "  Encoders : /joint_states  (L=%s  R=%s)",
                joint_left_.c_str(), joint_right_.c_str());
    RCLCPP_INFO(get_logger(), "  ZED2i    : /zed2i/zed_node/pose_with_covariance");
    RCLCPP_INFO(get_logger(), "  ArUco    : %s   (%s)",
                aruco_topic_.c_str(),
                aruco_enabled_ ? "ENABLED" : "DISABLED");
    RCLCPP_INFO(get_logger(), "  ArUco map: %zu markers loaded", aruco_map_.size());
    RCLCPP_INFO(get_logger(), "  ZUPT     : %s",
                zupt_enabled_ ? "ENABLED" : "DISABLED");
    RCLCPP_INFO(get_logger(), "  Output   : %s  (TF %s)",
                odom_topic_.c_str(), publish_tf_ ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(), "  TF root  : %s → %s  (static, %s)",
                map_frame_id_.c_str(), frame_id_.c_str(),
                publish_map_to_odom_ ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(), "  IMU mount offset (deg): roll=%.1f pitch=%.1f yaw=%.1f",
                this->get_parameter("imu_roll_deg").as_double(),
                this->get_parameter("imu_pitch_deg").as_double(),
                this->get_parameter("imu_yaw_deg").as_double());
    RCLCPP_INFO(get_logger(), "  ZED→base (body frame): [%.3f, %.3f, %.3f] m",
                t_cam_in_body_.x(), t_cam_in_body_.y(), t_cam_in_body_.z());
}

// ─────────────────────────────────────────────────────────────────────────────
// loadParams
// ─────────────────────────────────────────────────────────────────────────────
void IESKFNode::loadParams()
{
    // ── Frame IDs ─────────────────────────────────────────────────────────────
    frame_id_       = declare_parameter<std::string>("frame_id",       "odom");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    publish_tf_     = declare_parameter<bool>       ("publish_tf",     true);

    map_frame_id_          = declare_parameter<std::string>("map_frame_id", "map");
    publish_map_to_odom_   = declare_parameter<bool>       ("publish_map_to_odom", true);

    // ── Output topic ──────────────────────────────────────────────────────────
    odom_topic_     = declare_parameter<std::string>("odom_topic",  "/odom");

    // ── ArUco topic ───────────────────────────────────────────────────────────
    aruco_topic_    = declare_parameter<std::string>("aruco_topic", "/aruco/detections");
    aruco_enabled_  = declare_parameter<bool>       ("aruco_enabled", true);
    aruco_max_distance_ =
        declare_parameter<double>("aruco_max_distance", 15.0);

    // ── ArUco map file ────────────────────────────────────────────────────────
    // If empty, no markers are loaded (ArUco update path becomes a no-op).
    // Otherwise try to load YAML; failures are logged but non-fatal.
    const std::string aruco_map_path =
        declare_parameter<std::string>("aruco_map_path", "");
    if (!aruco_map_path.empty()) {
        try {
            aruco_map_.loadFromYaml(aruco_map_path);
            RCLCPP_INFO(get_logger(), "Loaded ArUco map: %s (%zu markers)",
                        aruco_map_path.c_str(), aruco_map_.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load ArUco map: %s", e.what());
            RCLCPP_WARN(get_logger(), "Continuing without ArUco corrections.");
        }
    } else {
        RCLCPP_WARN(get_logger(),
            "No aruco_map_path provided — ArUco updates will be skipped.");
    }

    // ── Joint names ───────────────────────────────────────────────────────────
    joint_left_  = declare_parameter<std::string>("joint_left",  "wheel_left_joint");
    joint_right_ = declare_parameter<std::string>("joint_right", "wheel_right_joint");

    // ── WIO ───────────────────────────────────────────────────────────────────
    WIOParams wio_p;
    wio_p.wheel_radius     = declare_parameter<double>("wheel_radius",     0.1375);
    wio_p.wheelbase        = declare_parameter<double>("wheelbase",        0.5);
    wio_p.pulses_per_rev   = declare_parameter<double>("pulses_per_rev",   2000.0);
    wio_p.max_vx           = declare_parameter<double>("max_vx",           2.0);
    wio_p.max_delta_pulses = declare_parameter<double>("max_delta_pulses", 500.0);
    wio_ = std::make_unique<WIO>(wio_p);

    // ── WVO ───────────────────────────────────────────────────────────────────
    WVOParams wvo_p;
    wvo_p.max_jump        = declare_parameter<double>("wvo_max_jump",   0.5);
    wvo_p.window_size     = declare_parameter<int>   ("wvo_window_size", 5);
    wvo_p.w_wo            = declare_parameter<double>("wvo_w_wo",       0.3);
    wvo_p.min_window_fill = declare_parameter<int>   ("wvo_min_fill",   3);
    wvo_ = std::make_unique<WVO>(wvo_p);

    // ── IESKF ─────────────────────────────────────────────────────────────────
    IESKFParams ieskf_p;
    ieskf_p.sigma_an         = declare_parameter<double>("sigma_an",         3.0e-3);
    ieskf_p.sigma_aw         = declare_parameter<double>("sigma_aw",         1.0e-4);
    ieskf_p.sigma_ww         = declare_parameter<double>("sigma_ww",         8.7e-5);
    ieskf_p.sigma_pos        = declare_parameter<double>("sigma_pos",        0.05);
    ieskf_p.sigma_aruco_base = declare_parameter<double>("sigma_aruco_base", 0.01);
    ieskf_p.sigma_aruco_per_m= declare_parameter<double>("sigma_aruco_per_m",0.005);
    ieskf_p.max_iterations   = declare_parameter<int>   ("max_iterations",   10);
    ieskf_p.epsilon          = declare_parameter<double>("epsilon",          1e-6);
    ieskf_p.max_velocity     = declare_parameter<double>("max_velocity",     5.0);
    filter_ = std::make_unique<IESKF>(ieskf_p);

    // ── IMU mounting correction ───────────────────────────────────────────────
    const double imu_roll_deg  = declare_parameter<double>("imu_roll_deg",  0.0);
    const double imu_pitch_deg = declare_parameter<double>("imu_pitch_deg", 0.0);
    const double imu_yaw_deg   = declare_parameter<double>("imu_yaw_deg",   0.0);
    const double d2r = M_PI / 180.0;

    const Quat q_mount =
        Eigen::AngleAxisd(imu_yaw_deg   * d2r, Vec3::UnitZ()) *
        Eigen::AngleAxisd(imu_pitch_deg * d2r, Vec3::UnitY()) *
        Eigen::AngleAxisd(imu_roll_deg  * d2r, Vec3::UnitX());
    R_imu_to_body_ = q_mount.toRotationMatrix();

    // ── ZED extrinsic ─────────────────────────────────────────────────────────
    t_cam_in_body_ = Vec3(
        declare_parameter<double>("cam_to_base_x", 0.0),
        declare_parameter<double>("cam_to_base_y", 0.0),
        declare_parameter<double>("cam_to_base_z", 0.0)
    );

    // R_cam_to_body: by default, ZED optical convention to body convention.
    // ZED optical: x=right, y=down, z=forward (from camera's perspective)
    // Body:        x=forward, y=left, z=up
    //
    // Transformation: (x_cam, y_cam, z_cam) → (z_cam, -x_cam, -y_cam) in body
    //
    //   R_cam_to_body =  [ 0,  0,  1 ]
    //                    [-1,  0,  0 ]
    //                    [ 0, -1,  0 ]
    //
    // An optional additional rotation (cam_roll/pitch/yaw_deg) can rotate
    // the camera further (e.g., if the ZED is tilted downward).
    Mat3 R_optical_to_body;
    R_optical_to_body <<  0.0,  0.0,  1.0,
                         -1.0,  0.0,  0.0,
                          0.0, -1.0,  0.0;

    const double cam_roll_deg  = declare_parameter<double>("cam_roll_deg",  0.0);
    const double cam_pitch_deg = declare_parameter<double>("cam_pitch_deg", 0.0);
    const double cam_yaw_deg   = declare_parameter<double>("cam_yaw_deg",   0.0);
    const Quat q_cam_extra =
        Eigen::AngleAxisd(cam_yaw_deg   * d2r, Vec3::UnitZ()) *
        Eigen::AngleAxisd(cam_pitch_deg * d2r, Vec3::UnitY()) *
        Eigen::AngleAxisd(cam_roll_deg  * d2r, Vec3::UnitX());
    R_cam_to_body_ = q_cam_extra.toRotationMatrix() * R_optical_to_body;

    // ── FIS ranges ────────────────────────────────────────────────────────────
    max_omega_z_   = declare_parameter<double>("max_omega_z",   1.5);
    max_vx_        = wio_p.max_vx;
    zed_cov_scale_ = declare_parameter<double>("zed_cov_scale", 0.01);

    fuzzy::WOFISParams wo_p;
    wo_p.k_Q_min         = declare_parameter<double>("wofis_k_q_min",        1.0);
    wo_p.k_Q_max         = declare_parameter<double>("wofis_k_q_max",       10.0);
    wo_p.slip_low_upper  = declare_parameter<double>("wofis_slip_low",       0.15);
    wo_p.slip_med_peak   = declare_parameter<double>("wofis_slip_med_peak",  0.25);
    wo_p.slip_high_lower = declare_parameter<double>("wofis_slip_high",      0.35);
    wo_fis_ = fuzzy::WOFIS(wo_p);

    fuzzy::VOFISParams vo_p;
    vo_p.k_V_min         = declare_parameter<double>("vofis_k_v_min",         1.0);
    vo_p.k_V_max         = declare_parameter<double>("vofis_k_v_max",       100.0);
    vo_p.conf_low_upper  = declare_parameter<double>("vofis_conf_low",        40.0);
    vo_p.conf_med_peak   = declare_parameter<double>("vofis_conf_med_peak",   55.0);
    vo_p.conf_high_lower = declare_parameter<double>("vofis_conf_high",       65.0);
    vo_p.jump_cons_upper = declare_parameter<double>("vofis_jump_consistent", 1.5);
    vo_p.jump_div_peak   = declare_parameter<double>("vofis_jump_div_peak",   2.5);
    vo_p.jump_unrel_lower= declare_parameter<double>("vofis_jump_unreliable", 3.0);
    vo_fis_ = fuzzy::VOFIS(vo_p);

    // ── ZUPT ──────────────────────────────────────────────────────────────────
    zupt_enabled_ = declare_parameter<bool>("zupt_enabled", true);

    ZuptParams zupt_p;
    zupt_p.encoder_speed_threshold =
        declare_parameter<double>("zupt_encoder_speed_threshold", 0.02);
    zupt_p.angular_velocity_threshold =
        declare_parameter<double>("zupt_angular_velocity_threshold", 0.05);
    zupt_p.accel_variance_threshold =
        declare_parameter<double>("zupt_accel_variance_threshold", 0.05);
    zupt_p.accel_buffer_size =
        declare_parameter<int>("zupt_accel_buffer_size", 30);
    zupt_p.min_stationary_duration =
        declare_parameter<double>("zupt_min_stationary_duration", 0.5);
    zupt_p.sigma_v_zupt =
        declare_parameter<double>("zupt_sigma_v", 0.01);
    zupt_ = std::make_unique<ZuptDetector>(zupt_p);

    RCLCPP_INFO(get_logger(),
        "Params: wheel_r=%.4f ppr=%.0f sigma_an=%.2e sigma_pos=%.3f "
        "sigma_aruco_base=%.3f",
        wio_p.wheel_radius, wio_p.pulses_per_rev,
        ieskf_p.sigma_an, ieskf_p.sigma_pos,
        ieskf_p.sigma_aruco_base);
}

// ─────────────────────────────────────────────────────────────────────────────
// jointStateCallback — store encoder counts
// ─────────────────────────────────────────────────────────────────────────────
void IESKFNode::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == joint_left_  && i < msg->position.size())
            encoder_left_  = msg->position[i];
        if (msg->name[i] == joint_right_ && i < msg->position.size())
            encoder_right_ = msg->position[i];
    }
    encoder_ready_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// imuCallback — predict step + ZUPT (~100 Hz)
// ─────────────────────────────────────────────────────────────────────────────
void IESKFNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const rclcpp::Time now = this->get_clock()->now();

    // ── dt ────────────────────────────────────────────────────────────────────
    if (first_imu_) {
        last_imu_time_ = now;
        first_imu_     = false;
        return;
    }
    const double dt = (now - last_imu_time_).seconds();
    last_imu_time_  = now;

    if (dt <= 0.0 || dt > 0.5) {
        RCLCPP_WARN(get_logger(), "Skipping IMU step: dt=%.4f s", dt);
        return;
    }

    // ── Raw IMU data (still in IMU frame) ────────────────────────────────────
    const Vec3 a_m_imu(msg->linear_acceleration.x,
                       msg->linear_acceleration.y,
                       msg->linear_acceleration.z);
    const Vec3 omega_m_imu(msg->angular_velocity.x,
                           msg->angular_velocity.y,
                           msg->angular_velocity.z);
    const Quat q_imu_raw(msg->orientation.w,
                         msg->orientation.x,
                         msg->orientation.y,
                         msg->orientation.z);

    // ── IMU mounting correction → body frame ─────────────────────────────────
    const Vec3 a_m     = R_imu_to_body_ * a_m_imu;
    const Vec3 omega_m = R_imu_to_body_ * omega_m_imu;
    const Quat q_body  = (q_imu_raw * Quat(R_imu_to_body_.transpose())).normalized();
    current_orientation_ = q_body;

    // ── WIO ───────────────────────────────────────────────────────────────────
    const double pulses_L = encoder_ready_ ? encoder_left_  : 0.0;
    const double pulses_R = encoder_ready_ ? encoder_right_ : 0.0;
    WIOOutput wio_out = wio_->compute(pulses_L, pulses_R, q_body, dt);

    // ── Update WVO dead-reckoning accumulator with failure tracking ──────────
    if (wio_out.valid) {
        accumulated_dp_wo_ += wio_out.dp_world;
        wio_fail_count_ = 0;
    } else {
        wio_fail_count_++;
        if (wio_fail_count_ >= kMaxWioFails) {
            accumulated_dp_wo_ = Vec3::Zero();
            wio_fail_count_    = 0;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "WIO: %d consecutive encoder failures — resetting dp accumulator",
                kMaxWioFails);
        }
    }

    // ── WO-FIS: compute k_Q ──────────────────────────────────────────────────
    double k_Q = 1.0;
    if (wio_out.valid) {
        const double slip_ratio   =
            fuzzy::computeSlipRatio(wio_out.v_L, wio_out.v_R);
        const double omega_z_norm =
            std::min(std::abs(omega_m.z()) / max_omega_z_, 1.0);
        const double vx_norm      =
            std::min(std::abs(wio_out.vx) / max_vx_, 1.0);
        k_Q = wo_fis_.evaluate(slip_ratio, omega_z_norm, vx_norm);
    } else {
        const double omega_z_norm =
            std::min(std::abs(omega_m.z()) / max_omega_z_, 1.0);
        k_Q = wo_fis_.evaluate(0.0, omega_z_norm, 0.0);
    }

    // ── IESKF predict ────────────────────────────────────────────────────────
    if (wio_out.valid) {
        filter_->predict(a_m, omega_m, wio_out.R, wio_out.vx,
                         wio_out.dp_world, dt, k_Q);
    } else {
        const Mat3 R_fb = quat_to_rot(q_body);
        filter_->predict(a_m, omega_m, R_fb, 0.0, Vec3::Zero(), dt, k_Q);
    }

    diag_last_k_Q_ = k_Q;

    // ── ZUPT ─────────────────────────────────────────────────────────────────
    // Feed the detector with the latest readings. If all conditions held for
    // long enough, fire a zero-velocity update.
    if (zupt_enabled_ && wio_out.valid) {
        zupt_->update(wio_out.v_L, wio_out.v_R, omega_m.z(), a_m, now.seconds());
        if (zupt_->isStationary()) {
            // Inject v=0 measurement. Logged sparingly to avoid spam.
            filter_->updateVelocity(Vec3::Zero(), zupt_->getParams().sigma_v_zupt);
            diag_zupt_fired_++;
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                "ZUPT fired (accel_var=%.4f)",
                zupt_->currentAccelVariance());
        }
    }

    // ── Publish ──────────────────────────────────────────────────────────────
    publishOdometry(now);
    if (publish_tf_) publishTF(now);

    // ── Diagnostics (decimated to ~10 Hz at 100 Hz IMU rate) ────────────────
    if (++diag_decimator_ >= 10) {
        diag_decimator_ = 0;
        std_msgs::msg::Float64MultiArray d;
        d.data = {
            static_cast<double>(diag_aruco_received_),
            static_cast<double>(diag_aruco_applied_),
            static_cast<double>(diag_zupt_fired_),
            filter_->getCovariance().block<3,3>(0,0).trace(),
            diag_last_k_Q_,
            diag_last_k_V_
        };
        pub_diag_->publish(d);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// zedPoseCallback — position update from WVO/ZED2i (~30 Hz)
// ─────────────────────────────────────────────────────────────────────────────
void IESKFNode::zedPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    const Vec3 p_zed_raw(msg->pose.pose.position.x,
                         msg->pose.pose.position.y,
                         msg->pose.pose.position.z);

    // ── ZED extrinsic correction: convert camera pose → base_link pose ───────
    // p_base = p_zed_world − R_body_to_world × t_cam_in_body
    const Mat3 R_cur  = quat_to_rot(current_orientation_);
    const Vec3 p_base = p_zed_raw - R_cur * t_cam_in_body_;

    // ── ZED confidence from covariance ───────────────────────────────────────
    const double cov_trace =
        msg->pose.covariance[0]  +
        msg->pose.covariance[7]  +
        msg->pose.covariance[14];
    const double zed_confidence =
        std::clamp(100.0 * std::exp(-cov_trace / zed_cov_scale_), 0.0, 100.0);

    // ── WVO ──────────────────────────────────────────────────────────────────
    WVOOutput wvo_out = wvo_->compute(p_base, accumulated_dp_wo_);
    accumulated_dp_wo_ = Vec3::Zero();

    // ── VO-FIS → k_V ─────────────────────────────────────────────────────────
    const double k_V = vo_fis_.evaluate(zed_confidence, wvo_out.jump_ratio);
    diag_last_k_V_ = k_V;

    // ── IESKF update ─────────────────────────────────────────────────────────
    if (wvo_out.valid && wvo_out.ready) {
        filter_->update(wvo_out.z, k_V);
    } else if (!wvo_out.valid) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "WVO: ZED tracking lost (jump_ratio=%.2f, confidence=%.1f) — skipping update",
            wvo_out.jump_ratio, zed_confidence);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// arucoCallback — landmark update for each detection in the array
// ─────────────────────────────────────────────────────────────────────────────
void IESKFNode::arucoCallback(
    const roar_msgs::msg::ArucoDetectionArray::SharedPtr msg)
{
    if (!aruco_enabled_ || aruco_map_.size() == 0) return;

    const Mat3 R_body_to_world = quat_to_rot(current_orientation_);
    const IESKFParams& fp      = filter_->getParams();

    int applied = 0;

    for (const auto& det : msg->detections)
    {
        diag_aruco_received_++;

        // ── Distance filter ──────────────────────────────────────────────────
        if (det.distance > aruco_max_distance_) continue;

        // ── Look up marker world position ────────────────────────────────────
        auto marker_world_opt = aruco_map_.getMarkerPosition(det.id);
        if (!marker_world_opt) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "ArUco id=%d not in map — ignoring", det.id);
            continue;
        }

        // ── Extract marker position in camera optical frame ──────────────────
        const Vec3 p_marker_camera(
            det.pose_in_camera.position.x,
            det.pose_in_camera.position.y,
            det.pose_in_camera.position.z);

        // ── Distance-scaled noise ────────────────────────────────────────────
        // sigma_aruco grows linearly with distance — pixel error projects
        // to larger world error as the marker is farther away.
        const double sigma_aruco =
            fp.sigma_aruco_base + fp.sigma_aruco_per_m * det.distance;

        // ── Apply update ─────────────────────────────────────────────────────
        filter_->updateAruco(
            *marker_world_opt,
            p_marker_camera,
            R_body_to_world,
            t_cam_in_body_,
            R_cam_to_body_,
            sigma_aruco);

        ++applied;
        diag_aruco_applied_++;
    }

    if (applied > 0) {
        RCLCPP_DEBUG(get_logger(), "Applied %d ArUco updates", applied);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// publishOdometry — same as before
// ─────────────────────────────────────────────────────────────────────────────
void IESKFNode::publishOdometry(const rclcpp::Time& stamp)
{
    const IESKFState& state = filter_->getState();
    const auto&       P     = filter_->getCovariance();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id  = child_frame_id_;

    odom.pose.pose.position.x = state.p.x();
    odom.pose.pose.position.y = state.p.y();
    odom.pose.pose.position.z = state.p.z();

    odom.pose.pose.orientation.x = current_orientation_.x();
    odom.pose.pose.orientation.y = current_orientation_.y();
    odom.pose.pose.orientation.z = current_orientation_.z();
    odom.pose.pose.orientation.w = current_orientation_.w();

    odom.twist.twist.linear.x = state.v.x();
    odom.twist.twist.linear.y = state.v.y();
    odom.twist.twist.linear.z = state.v.z();

    // 3×3 position covariance block
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            odom.pose.covariance[i * 6 + j] = P(i, j);

    // Small fixed orientation covariance (BNO055 NDOF ≈ 1°)
    const double ori_var = 0.017 * 0.017;
    odom.pose.covariance[3*6+3] = ori_var;
    odom.pose.covariance[4*6+4] = ori_var;
    odom.pose.covariance[5*6+5] = ori_var;

    // Velocity covariance
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            odom.twist.covariance[i * 6 + j] = P(3+i, 3+j);

    pub_odom_->publish(odom);
}

// ─────────────────────────────────────────────────────────────────────────────
// publishTF — broadcast odom → base_link
// ─────────────────────────────────────────────────────────────────────────────
void IESKFNode::publishTF(const rclcpp::Time& stamp)
{
    const IESKFState& state = filter_->getState();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = stamp;
    tf_msg.header.frame_id = frame_id_;
    tf_msg.child_frame_id  = child_frame_id_;

    tf_msg.transform.translation.x = state.p.x();
    tf_msg.transform.translation.y = state.p.y();
    tf_msg.transform.translation.z = state.p.z();
    tf_msg.transform.rotation.x = current_orientation_.x();
    tf_msg.transform.rotation.y = current_orientation_.y();
    tf_msg.transform.rotation.z = current_orientation_.z();
    tf_msg.transform.rotation.w = current_orientation_.w();

    tf_broadcaster_->sendTransform(tf_msg);
}

// ─────────────────────────────────────────────────────────────────────────────
// publishStaticMapToOdomTF — broadcast map → odom (once, on /tf_static)
//
// This node has no separate global localization estimate (ArUco corrections
// are fused straight into the odom → base_link state above), so there is no
// meaningful non-identity pose to put here. This purely roots /map above
// /odom in the TF tree. Published once at startup as a static transform (not
// re-sent on /tf every cycle) so it stays valid regardless of which clock
// domain publishes odom → base_link (e.g. Gazebo's diff_drive plugin uses
// its own sim clock in some launches). It does not read from or influence
// the IESKF state.
// ─────────────────────────────────────────────────────────────────────────────
void IESKFNode::publishStaticMapToOdomTF()
{
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = this->now();
    tf_msg.header.frame_id = map_frame_id_;
    tf_msg.child_frame_id  = frame_id_;

    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(tf_msg);
}

} // namespace roar

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<roar::IESKFNode>());
    rclcpp::shutdown();
    return 0;
}
