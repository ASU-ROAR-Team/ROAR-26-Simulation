#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_set>
#include <mutex>
#include <thread>
#include <vector>

namespace path_planner
{

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEps = 1e-6;

// To match original Costmap2D lethal/unknown values
constexpr unsigned char NO_INFORMATION = 255;
constexpr unsigned char LETHAL_OBSTACLE = 254;

class DStarGlobalPlannerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  DStarGlobalPlannerNode() : Node("dstar_global_planner")
  {
    // Parameters
    this->declare_parameter("lethal_cost_threshold", 253);
    this->declare_parameter("unknown_traversal_cost", 1.0);
    this->declare_parameter("allow_unknown", true);
    this->declare_parameter("heuristic_weight", 1.0);
    this->declare_parameter("enable_clearance_field", true);
    this->declare_parameter("clearance_field_weight", 3.0);
    this->declare_parameter("clearance_decay_distance", 0.5);
    this->declare_parameter("terrain_weight", 2.0);
    this->declare_parameter("enable_incremental_updates", true);
    this->declare_parameter("unified_costmap_topic", std::string("/unified_costmap"));

    // Debug Publishers
    this->declare_parameter("debug_publish_costmap", false);
    this->declare_parameter("debug_publish_potential", false);
    this->declare_parameter("debug_publish_priority", false);
    this->declare_parameter("debug_publish_cell_cost", false);

    loadParameters();

    // Load debug params (only needed at construction, publishers created once)
    this->get_parameter("debug_publish_costmap", debug_publish_costmap_);
    this->get_parameter("debug_publish_potential", debug_publish_potential_);
    this->get_parameter("debug_publish_priority", debug_publish_priority_);
    this->get_parameter("debug_publish_cell_cost", debug_publish_cell_cost_);

    // Publishers
    rclcpp::QoS plan_qos(1);
    plan_qos.transient_local().reliable();
    plan_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_plan", plan_qos);
    
    if (debug_publish_costmap_)
      costmap_viz_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("debug_costs", 1);
    if (debug_publish_potential_)
      potential_viz_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("debug_potential", 1);
    if (debug_publish_priority_) {
      key_k1_viz_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("debug_key_k1", 1);
      key_k2_viz_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("debug_key_k2", 1);
    }
    if (debug_publish_cell_cost_)
      cell_cost_viz_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("debug_cell_cost", 1);

    // Subscribers — subscribe to the unified costmap (fused SLAM + terrain)
    rclcpp::QoS map_qos(1);
    map_qos.transient_local().reliable();
    unified_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      unified_costmap_topic_, map_qos, std::bind(&DStarGlobalPlannerNode::unifiedCostmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 1, std::bind(&DStarGlobalPlannerNode::odomCallback, this, std::placeholders::_1));
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&DStarGlobalPlannerNode::goalPoseCallback, this, std::placeholders::_1));

    // Action Server
    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      "navigate_to_pose",
      std::bind(&DStarGlobalPlannerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DStarGlobalPlannerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&DStarGlobalPlannerNode::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "DStarGlobalPlannerNode initialized (allow_unknown=%s)", allow_unknown_ ? "true" : "false");
  }

private:
  // ── CORE DATA STRUCTURES ───────────────────────────────────────────────────
  struct Key { double k1; double k2; };
  struct QueueItem { Key key; unsigned int idx; };

  struct QueueCompare {
    bool operator()(const QueueItem& a, const QueueItem& b) const noexcept {
      if (std::fabs(a.key.k1 - b.key.k1) > kEps) return a.key.k1 > b.key.k1;
      return a.key.k2 > b.key.k2;
    }
  };

  struct NeighborList {
    unsigned int indices[8];
    int count = 0;
  };

  // ── VARIABLES ──────────────────────────────────────────────────────────────
  bool map_ready_ = false;
  bool pose_ready_ = false;
  bool map_changed_ = false;  // Flag for map-change-triggered replanning
  bool goal_active_ = false;  // Whether a NavigateToPose goal is currently being executed
  
  std::vector<unsigned char> costmap_data_;
  unsigned int width_ = 0;
  unsigned int height_ = 0;
  double resolution_ = 0.05;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  std::string global_frame_ = "map";

  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped active_goal_;  // Currently active goal pose
  std::mutex map_mutex_;
  std::mutex pose_mutex_;
  std::shared_ptr<GoalHandleNavigateToPose> current_goal_handle_;
  std::mutex goal_handle_mutex_;

  // Parameters
  int lethal_cost_threshold_;
  double unknown_traversal_cost_;
  bool allow_unknown_;
  double heuristic_weight_;
  bool enable_clearance_field_;
  double clearance_field_weight_;
  double clearance_decay_distance_;
  double terrain_weight_;
  bool enable_incremental_updates_;
  std::string unified_costmap_topic_ = "/unified_costmap";

  bool debug_publish_costmap_ = false;
  bool debug_publish_potential_ = false;
  bool debug_publish_priority_ = false;
  bool debug_publish_cell_cost_ = false;

  // D* State
  std::vector<double> g_;
  std::vector<double> rhs_;
  std::vector<bool> in_open_;
  std::vector<Key> open_key_;
  std::vector<unsigned char> last_costs_;
  std::vector<float> clearance_m_;
  std::priority_queue<QueueItem, std::vector<QueueItem>, QueueCompare> open_list_;

  double km_ = 0.0;
  unsigned int start_idx_ = std::numeric_limits<unsigned int>::max();
  unsigned int goal_idx_ = std::numeric_limits<unsigned int>::max();
  unsigned int last_start_idx_ = std::numeric_limits<unsigned int>::max();  // For km_ accumulation

  // ROS Interfaces
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr potential_viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr key_k1_viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr key_k2_viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cell_cost_viz_pub_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr unified_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

  // ── CALLBACKS ──────────────────────────────────────────────────────────────
  // Receives the fused costmap from heightmap_to_costmap: 100=lethal,
  // 0-99=terrain difficulty, -1=unknown.
  void unifiedCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    width_ = msg->info.width;
    height_ = msg->info.height;
    resolution_ = msg->info.resolution;
    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;
    global_frame_ = msg->header.frame_id;

    const size_t map_size = static_cast<size_t>(width_) * height_;
    if (msg->data.size() != map_size) {
      RCLCPP_WARN(this->get_logger(), "Unified costmap data size mismatch: %zu vs expected %zu",
                  msg->data.size(), map_size);
      return;
    }
    std::vector<unsigned char> new_costmap(map_size);

    for (size_t i = 0; i < map_size; ++i) {
      int8_t val = msg->data[i];
      if (val == -1) new_costmap[i] = NO_INFORMATION;
      else if (val >= 100) new_costmap[i] = LETHAL_OBSTACLE;
      else new_costmap[i] = static_cast<unsigned char>((std::max(static_cast<int8_t>(0), val) / 100.0) * 253.0);
    }

    // Only flag map_changed_ if data actually differs
    if (new_costmap != costmap_data_) {
      costmap_data_ = std::move(new_costmap);
      map_changed_ = true;  // Signal for map-change replanning
      RCLCPP_DEBUG(this->get_logger(), "Unified costmap updated: data changed.");
    }
    map_ready_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    pose_ready_ = true;
  }

  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!map_ready_ || !pose_ready_) {
      RCLCPP_WARN(this->get_logger(), "Cannot plan to /goal_pose: Map or Pose not yet received.");
      return;
    }
    geometry_msgs::msg::PoseStamped start_pose;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      start_pose = current_pose_;
    }
    RCLCPP_INFO(this->get_logger(), "Received /goal_pose (x=%.2f, y=%.2f). Computing D* path...",
                msg->pose.position.x, msg->pose.position.y);
    active_goal_ = *msg;
    goal_active_ = true;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map_changed_ = false;
    }
    std::vector<geometry_msgs::msg::PoseStamped> plan;
    bool success = makePlan(start_pose, *msg, plan);
    if (success) {
      RCLCPP_INFO(this->get_logger(), "/goal_pose path published with %zu poses.", plan.size());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to find path for /goal_pose.");
    }
  }

  // ── ACTION SERVER ──────────────────────────────────────────────────────────
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    (void)uuid;
    if (!map_ready_ || !pose_ready_) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: Map or Pose not yet received.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Goal request received.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Goal canceled.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(goal_handle_mutex_);
      current_goal_handle_ = goal_handle;
    }
    std::thread{std::bind(&DStarGlobalPlannerNode::execute_plan, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute_plan(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(goal_handle_mutex_);
      if (goal_handle != current_goal_handle_) {
        RCLCPP_INFO(this->get_logger(), "New goal accepted before thread started. Terminating.");
        return;
      }
    }
    auto result = std::make_shared<NavigateToPose::Result>();
    auto goal = goal_handle->get_goal();

    active_goal_ = goal->pose;
    goal_active_ = true;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map_changed_ = false;
    }

    geometry_msgs::msg::PoseStamped start_pose;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      start_pose = current_pose_;
    }

    // Initial plan
    std::vector<geometry_msgs::msg::PoseStamped> plan;
    bool success = makePlan(start_pose, goal->pose, plan);

    if (!success) {
      goal_active_ = false;
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "Failed to find initial path.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Initial path published with %zu poses.", plan.size());

    // Continuous replanning loop: replan when map changes, until robot reaches goal
    rclcpp::Rate rate(5.0);  // Check at 5 Hz
    while (rclcpp::ok()) {
      // Check if preempted by a newer goal request
      {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);
        if (goal_handle != current_goal_handle_) {
          RCLCPP_INFO(this->get_logger(), "Goal preempted. Terminating thread.");
          return;
        }
      }

      // Check for cancellation
      if (goal_handle->is_canceling()) {
        goal_active_ = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled.");
        return;
      }

      // Check if robot has reached the snapped goal
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        start_pose = current_pose_;
      }
      double target_x = goal->pose.pose.position.x;
      double target_y = goal->pose.pose.position.y;
      if (goal_idx_ != std::numeric_limits<unsigned int>::max()) {
        target_x = (goal_idx_ % width_) * resolution_ + origin_x_ + resolution_ / 2.0;
        target_y = (goal_idx_ / width_) * resolution_ + origin_y_ + resolution_ / 2.0;
      }
      const double dx = start_pose.pose.position.x - target_x;
      const double dy = start_pose.pose.position.y - target_y;
      if (std::hypot(dx, dy) < resolution_ * 2.0) {
        goal_active_ = false;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return;
      }

      // Replan if unified costmap has changed
      bool should_replan = false;
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        should_replan = map_changed_;
        map_changed_ = false;
      }
      if (should_replan) {
        plan.clear();
        success = makePlan(start_pose, goal->pose, plan);
        if (!success) {
          RCLCPP_WARN(this->get_logger(), "Replan failed after map change, keeping previous path.");
        } else {
          RCLCPP_INFO(this->get_logger(), "Replanned: %zu poses.", plan.size());
        }
      }

      // Publish feedback
      auto feedback = std::make_shared<NavigateToPose::Feedback>();
      feedback->current_pose = start_pose;
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    goal_active_ = false;
  }

  // ── CORE ALGORITHM PORT (Identical Logic) ──────────────────────────────────
  void loadParameters()
  {
    this->get_parameter("lethal_cost_threshold", lethal_cost_threshold_);
    this->get_parameter("unknown_traversal_cost", unknown_traversal_cost_);
    this->get_parameter("allow_unknown", allow_unknown_);
    this->get_parameter("heuristic_weight", heuristic_weight_);
    this->get_parameter("enable_clearance_field", enable_clearance_field_);
    this->get_parameter("clearance_field_weight", clearance_field_weight_);
    this->get_parameter("clearance_decay_distance", clearance_decay_distance_);
    this->get_parameter("terrain_weight", terrain_weight_);
    this->get_parameter("enable_incremental_updates", enable_incremental_updates_);
    this->get_parameter("unified_costmap_topic", unified_costmap_topic_);
  }

  bool makePlan(const geometry_msgs::msg::PoseStamped& start,
                const geometry_msgs::msg::PoseStamped& goal,
                std::vector<geometry_msgs::msg::PoseStamped>& plan)
  {
    plan.clear();
    std::lock_guard<std::mutex> lock(map_mutex_);

    loadParameters();

    const size_t current_map_size = static_cast<size_t>(width_) * height_;
    if (g_.size() != current_map_size) {
      RCLCPP_INFO(this->get_logger(), "Resizing map arrays to %dx%d.", width_, height_);
      g_.resize(current_map_size);
      rhs_.resize(current_map_size);
      in_open_.resize(current_map_size);
      open_key_.resize(current_map_size);
      last_costs_.assign(current_map_size, NO_INFORMATION);
      clearance_m_.assign(current_map_size, 0.0f);
      goal_idx_ = std::numeric_limits<unsigned int>::max(); 
    }

    unsigned int start_x, start_y, goal_x, goal_y;
    if (!worldToMapSafe(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
        !worldToMapSafe(goal.pose.position.x,  goal.pose.position.y,  goal_x,  goal_y))
    {
      RCLCPP_WARN(this->get_logger(), "Start or goal is outside global boundaries.");
      return false;
    }

    unsigned int snapped_start = toIndex(start_x, start_y);
    if (!isCellValid(static_cast<int>(start_x), static_cast<int>(start_y))) {
      double best_dist = kInf;
      bool found = false;
      int sx = static_cast<int>(start_x);
      int sy = static_cast<int>(start_y);
      for (int dx = -2; dx <= 2; ++dx) {
        for (int dy = -2; dy <= 2; ++dy) {
          if (dx == 0 && dy == 0) continue;
          int nx = sx + dx;
          int ny = sy + dy;
          if (nx >= 0 && ny >= 0 && nx < static_cast<int>(width_) && ny < static_cast<int>(height_)) {
            if (isCellValid(nx, ny)) {
              double dist = std::hypot(dx, dy);
              if (dist < best_dist) {
                best_dist = dist;
                snapped_start = toIndex(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
                found = true;
              }
            }
          }
        }
      }
      if (found) {
        RCLCPP_WARN(this->get_logger(), "Start pose (%d, %d) is invalid! Snapped to nearest valid cell (%d, %d)", 
                    sx, sy, snapped_start % width_, snapped_start / width_);
      } else {
        RCLCPP_WARN(this->get_logger(), "Start pose (%d, %d) is invalid and no valid neighbor found within 2 cells!", sx, sy);
        return false;
      }
    }

    const unsigned int new_start = snapped_start;

    unsigned int snapped_goal = toIndex(goal_x, goal_y);
    if (!isCellValid(static_cast<int>(goal_x), static_cast<int>(goal_y))) {
      RCLCPP_WARN(this->get_logger(), "Goal pose (%d, %d) is invalid! Searching for closest valid cell within 5.0m...", goal_x, goal_y);
      bool fallback_found = false;
      double best_dist = kInf;
      unsigned int best_idx = snapped_goal;

      std::queue<unsigned int> q;
      std::unordered_set<unsigned int> visited_fallback;
      q.push(snapped_goal);
      visited_fallback.insert(snapped_goal);

      while (!q.empty()) {
        unsigned int curr = q.front();
        q.pop();

        auto [cx, cy] = toXY(curr);
        double dist = std::hypot(static_cast<double>(cx) - static_cast<double>(goal_x),
                                 static_cast<double>(cy) - static_cast<double>(goal_y)) * resolution_;
        if (dist > 5.0) continue;

        if (isCellValid(static_cast<int>(cx), static_cast<int>(cy))) {
          if (dist < best_dist) {
            best_dist = dist;
            best_idx = curr;
            fallback_found = true;
            break;
          }
        }

        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            int nx = static_cast<int>(cx) + dx;
            int ny = static_cast<int>(cy) + dy;
            if (nx >= 0 && ny >= 0 && nx < static_cast<int>(width_) && ny < static_cast<int>(height_)) {
              unsigned int nidx = toIndex(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
              if (visited_fallback.count(nidx) == 0) {
                visited_fallback.insert(nidx);
                q.push(nidx);
              }
            }
          }
        }
      }

      if (fallback_found) {
        RCLCPP_INFO(this->get_logger(), "Fallback valid cell found at (%d, %d) with distance %.2fm", 
                    best_idx % width_, best_idx / width_, best_dist);
        snapped_goal = best_idx;
      } else {
        RCLCPP_WARN(this->get_logger(), "No valid fallback cell found within 5.0m radius of goal!");
      }
    }

    const unsigned int new_goal  = snapped_goal;
    const unsigned char* current_costs = costmap_data_.data();

    if (enable_clearance_field_ && clearance_m_.size() == current_map_size)
      computeClearanceField(current_costs, current_map_size);

    const bool goal_changed = (new_goal != goal_idx_);
    const bool start_changed = (new_start != start_idx_);
    const bool can_incremental = enable_incremental_updates_ &&
                                 !goal_changed &&
                                 last_costs_.size() == current_map_size;

    if (goal_changed) {
      // Goal changed: full re-initialization required
      start_idx_ = new_start;
      goal_idx_  = new_goal;
      last_start_idx_ = start_idx_;
      initializePlannerState();
      computeShortestPath();
    }
    else if (can_incremental) {
      // D* Lite km_ accumulation: compensate for robot movement
      // km += h(s_last, s_new_start) per Koenig & Likhachev 2002
      if (start_changed) {
        km_ += heuristic(last_start_idx_, new_start);
        start_idx_ = new_start;
        last_start_idx_ = start_idx_;
      }

      // Diff costmap and update changed vertices
      for (size_t i = 0; i < current_map_size; ++i) {
        if (last_costs_[i] == current_costs[i]) continue;
        const unsigned int idx = static_cast<unsigned int>(i);
        updateVertex(idx);
        const NeighborList nbrs = getNeighbors(idx);
        for (int k = 0; k < nbrs.count; ++k)
          updateVertex(nbrs.indices[k]);
      }
      computeShortestPath();
    }
    else {
      // Start changed but incremental disabled, or first plan with this goal
      start_idx_ = new_start;
      goal_idx_  = new_goal;
      last_start_idx_ = start_idx_;
      initializePlannerState();
      computeShortestPath();
    }

    if (!std::isfinite(g_[start_idx_])) {
      RCLCPP_WARN(this->get_logger(), "No D* Lite path found to goal");
      return false;
    }

    // Path Extraction (IDENTICAL to original)
    plan.reserve(static_cast<size_t>(width_ + height_));
    if (new_start == toIndex(start_x, start_y)) {
      plan.push_back(start);
    } else {
      geometry_msgs::msg::PoseStamped snapped_pose = start;
      mapToWorldPose(new_start % width_, new_start / width_, snapped_pose);
      plan.push_back(snapped_pose);
    }

    unsigned int current_idx = start_idx_;
    unsigned int prev_idx    = std::numeric_limits<unsigned int>::max();
    const size_t max_steps   = static_cast<size_t>(width_) * height_;
    
    std::unordered_set<unsigned int> visited;
    visited.insert(start_idx_);

    for (size_t step = 0; step < max_steps && current_idx != goal_idx_; ++step)
    {
      const NeighborList succs = getNeighbors(current_idx);
      double best = kInf;
      unsigned int next_idx = current_idx;

      auto consider = [&](unsigned int s, bool allow_visited) {
        if (!allow_visited && visited.count(s)) return;
        const double g_s = g_[s];
        if (!std::isfinite(g_s)) return;

        double candidate = edgeCost(current_idx, s) + g_s;
        if (s == prev_idx) candidate += 1e-3;

        if (candidate < best) {
          best = candidate;
          next_idx = s;
        }
      };

      for (int i = 0; i < succs.count; ++i) consider(succs.indices[i], false);

      if (next_idx == current_idx) {
        for (int i = 0; i < succs.count; ++i) consider(succs.indices[i], true);
      }

      if (next_idx == current_idx) {
        RCLCPP_WARN(this->get_logger(), "D* Lite extraction stuck");
        return false;
      }

      // A* Fallback (IDENTICAL logic)
      if (visited.count(next_idx)) {
        const size_t N = current_map_size;
        std::vector<int> parent(N, -1);
        std::vector<double> gscore(N, kInf);
        std::vector<bool> closed(N, false);

        struct ANode { double f; unsigned int idx; };
        struct AGreater { bool operator()(const ANode& a, const ANode& b) const { return a.f > b.f; } };
        std::priority_queue<ANode, std::vector<ANode>, AGreater> open;

        gscore[current_idx] = 0.0;
        open.push({heuristic(current_idx, goal_idx_), current_idx});

        bool found = false;
        while (!open.empty()) {
          const ANode n = open.top();
          open.pop();
          const unsigned int u = n.idx;
          if (closed[u]) continue;
          closed[u] = true;
          if (u == goal_idx_) { found = true; break; }

          const NeighborList nbrs = getNeighbors(u);
          for (int i = 0; i < nbrs.count; ++i) {
            const unsigned int v = nbrs.indices[i];
            if (closed[v]) continue;
            const double tentative = gscore[u] + edgeCost(u, v);
            if (tentative + 1e-9 < gscore[v]) {
              gscore[v] = tentative;
              parent[v] = static_cast<int>(u);
              open.push({tentative + heuristic(v, goal_idx_), v});
            }
          }
        }

        if (!found) {
          RCLCPP_WARN(this->get_logger(), "A* fallback failed");
          return false;
        }

        std::vector<unsigned int> seq;
        unsigned int cur = goal_idx_;
        while (cur != current_idx && parent[cur] >= 0) {
          seq.push_back(cur);
          cur = static_cast<unsigned int>(parent[cur]);
        }
        if (cur != current_idx) {
          RCLCPP_WARN(this->get_logger(), "A* fallback trace failed to connect to current_idx");
          return false;
        }
        std::reverse(seq.begin(), seq.end());

        for (unsigned int idx : seq) {
          auto [nx, ny] = toXY(idx);
          geometry_msgs::msg::PoseStamped pose;
          pose.header.frame_id = global_frame_;
          pose.header.stamp    = this->now();
          mapToWorldPose(nx, ny, pose);
          plan.push_back(pose);
        }
        current_idx = goal_idx_;
        break;
      }

      visited.insert(next_idx);
      auto [nx, ny] = toXY(next_idx);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.header.stamp    = this->now();
      mapToWorldPose(nx, ny, pose);
      plan.push_back(pose);
      prev_idx = current_idx;
      current_idx = next_idx;
    }

    if (current_idx != goal_idx_) {
      RCLCPP_WARN(this->get_logger(), "Extraction exceeded max steps");
      return false;
    }

    if (plan.size() >= 2) {
      if (new_start == toIndex(start_x, start_y)) {
        plan.front() = start;
      }
      if (new_goal == toIndex(goal_x, goal_y)) {
        plan.back()  = goal;
      }
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = global_frame_;
    path_msg.header.stamp    = this->now();
    path_msg.poses           = plan;
    plan_pub_->publish(path_msg);

    std::copy(current_costs, current_costs + current_map_size, last_costs_.begin());
    return true;
  }

  void initializePlannerState()
  {
    const size_t map_size = static_cast<size_t>(width_) * height_;
    g_.assign(map_size, kInf);
    rhs_.assign(map_size, kInf);
    in_open_.assign(map_size, false);
    open_key_.assign(map_size, {kInf, kInf});
    open_list_ = {};
    km_        = 0.0;

    rhs_[goal_idx_] = 0.0;
    const Key goal_key = calculateKey(goal_idx_);
    open_list_.push({goal_key, goal_idx_});
    in_open_[goal_idx_]  = true;
    open_key_[goal_idx_] = goal_key;

    std::copy(costmap_data_.begin(), costmap_data_.end(), last_costs_.begin());
  }

  void computeClearanceField(const unsigned char* costs, size_t map_size)
  {
    const float inf = std::numeric_limits<float>::infinity();
    std::fill(clearance_m_.begin(), clearance_m_.end(), inf);

    using Item = std::pair<float, unsigned int>; 
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;

    for (size_t i = 0; i < map_size; ++i) {
      // Only seed from actual lethal obstacles, NOT from unknown (NO_INFORMATION=255)
      if (costs[i] >= LETHAL_OBSTACLE && costs[i] != NO_INFORMATION) {
        clearance_m_[i] = 0.0f;
        pq.push({0.0f, static_cast<unsigned int>(i)});
      }
    }
    if (pq.empty()) {
      std::fill(clearance_m_.begin(), clearance_m_.end(), 1e6f);
      return;
    }

    while (!pq.empty()) {
      const auto [d, idx] = pq.top();
      pq.pop();
      if (d > clearance_m_[idx] + 1e-6f) continue;

      const auto [x, y] = toXY(idx);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          const int nx = static_cast<int>(x) + dx;
          const int ny = static_cast<int>(y) + dy;
          if (nx < 0 || ny < 0 || nx >= static_cast<int>(width_) || ny >= static_cast<int>(height_))
            continue;
          const unsigned int nidx = toIndex(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
          const float step = static_cast<float>(std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * resolution_);
          const float nd = d + step;
          if (nd + 1e-6f < clearance_m_[nidx]) {
            clearance_m_[nidx] = nd;
            pq.push({nd, nidx});
          }
        }
      }
    }
  }

  void computeShortestPath()
  {
    auto t_start = std::chrono::steady_clock::now();
    size_t expansions = 0;

    while (!open_list_.empty()) {
      const Key start_key = calculateKey(start_idx_);
      if (!keyLess(topKey(), start_key) && std::fabs(g_[start_idx_] - rhs_[start_idx_]) <= kEps)
        break;

      QueueItem top = open_list_.top();
      open_list_.pop();

      if (!in_open_[top.idx]) continue;
      if (!keyEqual(top.key, open_key_[top.idx])) continue;

      const Key new_key = calculateKey(top.idx);
      if (keyLess(top.key, new_key)) {
        open_list_.push({new_key, top.idx});
        open_key_[top.idx] = new_key;
        continue;
      }

      ++expansions;
      in_open_[top.idx] = false;
      const double g_u   = g_[top.idx];
      const double rhs_u = rhs_[top.idx];

      if (g_u > rhs_u) {
        g_[top.idx] = rhs_u;
        const NeighborList preds = getNeighbors(top.idx);
        for (int i = 0; i < preds.count; ++i) updateVertex(preds.indices[i]);
      } else {
        g_[top.idx] = kInf;
        updateVertex(top.idx);
        const NeighborList preds = getNeighbors(top.idx);
        for (int i = 0; i < preds.count; ++i) updateVertex(preds.indices[i]);
      }
    }

    auto t_end = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    RCLCPP_INFO(this->get_logger(), "computeShortestPath: %.2f ms, %zu expansions", elapsed_ms, expansions);
  }

  void updateVertex(unsigned int idx)
  {
    if (idx != goal_idx_) {
      const NeighborList succs = getNeighbors(idx);
      double best_rhs = kInf;
      for (int i = 0; i < succs.count; ++i) {
        const double g_s = g_[succs.indices[i]];
        if (std::isfinite(g_s))
          best_rhs = std::min(best_rhs, edgeCost(idx, succs.indices[i]) + g_s);
      }
      rhs_[idx] = best_rhs;
    }

    in_open_[idx] = false;
    if (std::fabs(g_[idx] - rhs_[idx]) > kEps) {
      const Key key = calculateKey(idx);
      open_list_.push({key, idx});
      in_open_[idx]  = true;
      open_key_[idx] = key;
    }
  }

  Key calculateKey(unsigned int idx) const {
    const double min_val = std::min(g_[idx], rhs_[idx]);
    return {min_val + heuristic(start_idx_, idx) + km_, min_val};
  }

  Key topKey() const noexcept {
    return open_list_.empty() ? Key{kInf, kInf} : open_list_.top().key;
  }

  bool keyLess(const Key& a, const Key& b) const noexcept {
    if (a.k1 < b.k1 - kEps) return true;
    if (std::fabs(a.k1 - b.k1) <= kEps && a.k2 < b.k2 - kEps) return true;
    return false;
  }

  bool keyEqual(const Key& a, const Key& b) const noexcept {
    return std::fabs(a.k1 - b.k1) <= kEps && std::fabs(a.k2 - b.k2) <= kEps;
  }

  bool isCellValid(int x, int y) const noexcept {
    if (x < 0 || y < 0 || x >= static_cast<int>(width_) || y >= static_cast<int>(height_))
      return false;
    const unsigned char cost = costmap_data_[toIndex(static_cast<unsigned int>(x), static_cast<unsigned int>(y))];
    // CRITICAL: Check NO_INFORMATION (255) BEFORE LETHAL_OBSTACLE (254)
    // because 255 >= 254, the old ordering always treated unknown as lethal.
    if (cost == NO_INFORMATION) return allow_unknown_;
    if (cost >= LETHAL_OBSTACLE) return false;
    return static_cast<int>(cost) < lethal_cost_threshold_;
  }

  // Neighbors: 8-connected grid. All lethal detection (slopes, cliffs, obstacles)
  // is now handled by the unified costmap publisher — cells marked 100 there become
  // LETHAL_OBSTACLE here and are rejected by isCellValid().
  NeighborList getNeighbors(unsigned int idx) const noexcept {
    const auto [x, y] = toXY(idx);
    NeighborList result;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        const int nx = static_cast<int>(x) + dx;
        const int ny = static_cast<int>(y) + dy;
        if (!isCellValid(nx, ny)) continue;
        result.indices[result.count++] = toIndex(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
      }
    }
    return result;
  }

  std::pair<double, double> cellWorld(unsigned int idx) const noexcept {
    const auto [mx, my] = toXY(idx);
    return {origin_x_ + (mx + 0.5) * resolution_, origin_y_ + (my + 0.5) * resolution_};
  }

  // ── Edge cost function ───────────────────────────────────────────────────
  // Two terms only:
  //   1. clearance_field_term: exponential decay from lethal obstacles (Dijkstra-based)
  //   2. terrain_term: terrain difficulty from unified costmap (0-99 → 0-252 internal)
  double edgeCost(unsigned int from_idx, unsigned int to_idx) const noexcept {
    const auto [fx, fy] = toXY(from_idx);
    const auto [tx, ty] = toXY(to_idx);
    const double dist = std::hypot(static_cast<double>(fx) - static_cast<double>(tx), static_cast<double>(fy) - static_cast<double>(ty));
    const unsigned char cost = costmap_data_[to_idx];
    if (cost == NO_INFORMATION) return dist * unknown_traversal_cost_;

    // Clearance field: exponential decay from lethal obstacles (Dijkstra-based)
    double clearance_field_term = 0.0;
    if (enable_clearance_field_ && clearance_field_weight_ > 0.0 && to_idx < clearance_m_.size()) {
      const double c_m = std::max(0.0, static_cast<double>(clearance_m_[to_idx]));
      clearance_field_term = clearance_field_weight_ * std::exp(-c_m / clearance_decay_distance_);
    }

    // Terrain cost: non-lethal cells carry terrain difficulty (0-252 from unified costmap)
    const double normalized_terrain = std::max(0.0, std::min(1.0, static_cast<double>(cost) / 252.0));
    const double terrain_term = terrain_weight_ * normalized_terrain * normalized_terrain;

    return dist * (1.0 + clearance_field_term + terrain_term);
  }

  double heuristic(unsigned int a_idx, unsigned int b_idx) const noexcept {
    const auto [ax, ay] = toXY(a_idx);
    const auto [bx, by] = toXY(b_idx);
    return std::hypot(static_cast<double>(ax) - static_cast<double>(bx), static_cast<double>(ay) - static_cast<double>(by)) * heuristic_weight_; 
  }

  bool worldToMapSafe(double wx, double wy, unsigned int& mx, unsigned int& my) const {
    if (wx < origin_x_ || wy < origin_y_) return false;
    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
    return (mx < width_ && my < height_);
  }

  void mapToWorldPose(unsigned int mx, unsigned int my, geometry_msgs::msg::PoseStamped& pose) const {
    pose.pose.position.x = origin_x_ + (mx + 0.5) * resolution_;
    pose.pose.position.y = origin_y_ + (my + 0.5) * resolution_;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
  }

  unsigned int toIndex(unsigned int x, unsigned int y) const noexcept {
    return y * width_ + x;
  }

  std::pair<unsigned int, unsigned int> toXY(unsigned int idx) const noexcept {
    return {idx % width_, idx / width_};
  }
};

}  // namespace path_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<path_planner::DStarGlobalPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}