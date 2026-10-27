#include "nav2_core/global_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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
#include <memory>

namespace nav2_dstar_planner
{

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEps = 1e-6;

constexpr unsigned char NO_INFORMATION = 255;
constexpr unsigned char LETHAL_OBSTACLE = 254;

class DStarPlanner : public nav2_core::GlobalPlanner
{
public:
  DStarPlanner() = default;
  ~DStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("Unable to lock parent node pointer");
    }

    tf_ = tf;
    costmap_ros_ = costmap_ros;
    name_ = name;
    logger_ = node->get_logger();

    // Declare and retrieve parameters
    node->declare_parameter(name + ".lethal_cost_threshold", 253);
    node->declare_parameter(name + ".unknown_traversal_cost", 1.0);
    node->declare_parameter(name + ".allow_unknown", true);
    node->declare_parameter(name + ".heuristic_weight", 1.0);
    node->declare_parameter(name + ".enable_clearance_field", true);
    node->declare_parameter(name + ".clearance_field_weight", 3.0);
    node->declare_parameter(name + ".clearance_decay_distance", 0.5);
    node->declare_parameter(name + ".terrain_weight", 2.0);
    node->declare_parameter(name + ".enable_incremental_updates", true);
    node->declare_parameter(name + ".unified_costmap_topic", std::string("/unified_costmap"));

    node->get_parameter(name + ".lethal_cost_threshold", lethal_cost_threshold_);
    node->get_parameter(name + ".unknown_traversal_cost", unknown_traversal_cost_);
    node->get_parameter(name + ".allow_unknown", allow_unknown_);
    node->get_parameter(name + ".heuristic_weight", heuristic_weight_);
    node->get_parameter(name + ".enable_clearance_field", enable_clearance_field_);
    node->get_parameter(name + ".clearance_field_weight", clearance_field_weight_);
    node->get_parameter(name + ".clearance_decay_distance", clearance_decay_distance_);
    node->get_parameter(name + ".terrain_weight", terrain_weight_);
    node->get_parameter(name + ".enable_incremental_updates", enable_incremental_updates_);
    node->get_parameter(name + ".unified_costmap_topic", unified_costmap_topic_);

    // Subscribe to the unified costmap (fused SLAM + terrain from heightmap_to_costmap)
    rclcpp::QoS map_qos(1);
    map_qos.transient_local().reliable();
    unified_costmap_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      unified_costmap_topic_, map_qos, std::bind(&DStarPlanner::unifiedCostmapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(logger_, "D* Lite Nav2 Planner Plugin configured successfully under name: %s", name.c_str());
  }

  void cleanup() override
  {
    RCLCPP_INFO(logger_, "Cleaning up D* Lite Planner plugin");
    unified_costmap_sub_.reset();
    costmap_data_.clear();
    last_costs_.clear();
    clearance_m_.clear();
    g_.clear();
    rhs_.clear();
    in_open_.clear();
    open_key_.clear();
    open_list_ = {};
  }

  void activate() override
  {
    RCLCPP_INFO(logger_, "Activating D* Lite Planner plugin");
  }

  void deactivate() override
  {
    RCLCPP_INFO(logger_, "Deactivating D* Lite Planner plugin");
  }


  // ── Path smoothing (moving-average, 5-point window) ──────────────────
  std::vector<geometry_msgs::msg::PoseStamped> smoothPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & in,
    int window)
  {
    if (in.size() < 3) return in;
    std::vector<geometry_msgs::msg::PoseStamped> out = in;
    int half = window / 2;
    for (int i = half; i < static_cast<int>(in.size()) - half; ++i) {
      double sx = 0, sy = 0, w_sum = 0;
      for (int j = -half; j <= half; ++j) {
        double w = 1.0 / (std::abs(j) + 1);
        sx += w * in[i + j].pose.position.x;
        sy += w * in[i + j].pose.position.y;
        w_sum += w;
      }
      out[i].pose.position.x = sx / w_sum;
      out[i].pose.position.y = sy / w_sum;
    }
    // Preserve start and goal exactly
    out.front() = in.front();
    out.back()  = in.back();
    return out;
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    nav_msgs::msg::Path plan;
    plan.header.frame_id = global_frame_;
    plan.header.stamp = rclcpp::Clock().now();

    if (!map_ready_) {
      RCLCPP_WARN(logger_, "Cannot plan path: No unified costmap received yet.");
      return plan;
    }

    const size_t current_map_size = static_cast<size_t>(width_) * height_;
    if (g_.size() != current_map_size) {
      RCLCPP_INFO(logger_, "Resizing planner map arrays to %dx%d.", width_, height_);
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
      RCLCPP_WARN(logger_, "Start or goal is outside global map boundaries.");
      return plan;
    }

    // 1. Snap Start Pose to nearest valid neighbor if it is in collision
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
        RCLCPP_WARN(logger_, "Start pose (%d, %d) is invalid! Snapped to nearest valid cell (%d, %d)", 
                    sx, sy, snapped_start % width_, snapped_start / width_);
      } else {
        RCLCPP_WARN(logger_, "Start pose (%d, %d) is invalid and no valid neighbor found within 2 cells!", sx, sy);
        return plan;
      }
    }

    // 2. Snap Goal Pose using BFS up to 5.0m radius if it is in collision
    unsigned int snapped_goal = toIndex(goal_x, goal_y);
    if (!isCellValid(static_cast<int>(goal_x), static_cast<int>(goal_y))) {
      RCLCPP_WARN(logger_, "Goal pose (%d, %d) is invalid! Searching for closest valid cell within 5.0m...", goal_x, goal_y);
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

        // Add neighbors to BFS queue
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
        RCLCPP_INFO(logger_, "Fallback valid cell found at (%d, %d) with distance %.2fm", 
                    best_idx % width_, best_idx / width_, best_dist);
        snapped_goal = best_idx;
      } else {
        RCLCPP_WARN(logger_, "No valid fallback cell found within 5.0m radius of goal!");
        return plan; // Returns empty path (failure)
      }
    }

    const unsigned int new_start = snapped_start;
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
      start_idx_ = new_start;
      goal_idx_  = new_goal;
      last_start_idx_ = start_idx_;
      initializePlannerState();
      computeShortestPath();
    }

    if (!std::isfinite(g_[start_idx_])) {
      RCLCPP_WARN(logger_, "No D* Lite path found to goal");
      return plan;
    }

    // Path Extraction
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    poses.reserve(static_cast<size_t>(width_ + height_));
    if (new_start == toIndex(start_x, start_y)) {
      poses.push_back(start);
    } else {
      geometry_msgs::msg::PoseStamped snapped_pose = start;
      mapToWorldPose(new_start % width_, new_start / width_, snapped_pose);
      poses.push_back(snapped_pose);
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
        RCLCPP_WARN(logger_, "D* Lite extraction stuck");
        return plan;
      }

      // A* Fallback
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
          RCLCPP_WARN(logger_, "A* fallback failed");
          return plan;
        }

        std::vector<unsigned int> seq;
        unsigned int cur = goal_idx_;
        while (cur != current_idx && parent[cur] >= 0) {
          seq.push_back(cur);
          cur = static_cast<unsigned int>(parent[cur]);
        }
        if (cur != current_idx) {
          RCLCPP_WARN(logger_, "A* fallback trace failed to connect to current_idx");
          return plan;
        }
        std::reverse(seq.begin(), seq.end());

        for (unsigned int idx : seq) {
          auto [nx, ny] = toXY(idx);
          geometry_msgs::msg::PoseStamped pose;
          pose.header.frame_id = global_frame_;
          pose.header.stamp    = rclcpp::Clock().now();
          mapToWorldPose(nx, ny, pose);
          poses.push_back(pose);
        }
        current_idx = goal_idx_;
        break;
      }

      visited.insert(next_idx);
      auto [nx, ny] = toXY(next_idx);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.header.stamp    = rclcpp::Clock().now();
      mapToWorldPose(nx, ny, pose);
      poses.push_back(pose);
      prev_idx = current_idx;
      current_idx = next_idx;
    }

    if (current_idx != goal_idx_) {
      RCLCPP_WARN(logger_, "Extraction exceeded max steps");
      return plan;
    }

    if (poses.size() >= 2) {
      if (new_start == toIndex(start_x, start_y)) {
        poses.front() = start;
      }
      if (new_goal == toIndex(goal_x, goal_y)) {
        poses.back()  = goal;
      }
    }

    plan.poses = poses;

    // Smooth the path — 3 passes with a 5-point window
    for (int pass = 0; pass < 3; ++pass) {
      plan.poses = smoothPath(plan.poses, 5);
    }

    std::copy(current_costs, current_costs + current_map_size, last_costs_.begin());
    return plan;
  }

private:
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

  // ── Unified costmap callback ─────────────────────────────────────────────
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
      RCLCPP_WARN(logger_, "Unified costmap data size mismatch: %zu vs expected %zu",
                  msg->data.size(), map_size);
      return;
    }
    costmap_data_.resize(map_size);

    for (size_t i = 0; i < map_size; ++i) {
      int8_t val = msg->data[i];
      if (val == -1) costmap_data_[i] = NO_INFORMATION;
      else if (val >= 100) costmap_data_[i] = LETHAL_OBSTACLE;
      else costmap_data_[i] = static_cast<unsigned char>((std::max(static_cast<int8_t>(0), val) / 100.0) * 253.0);
    }
    map_ready_ = true;
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
    RCLCPP_INFO(logger_, "computeShortestPath: %.2f ms, %zu expansions", elapsed_ms, expansions);
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

  // Member variables
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("DStarPlanner")};

  bool map_ready_ = false;
  std::mutex map_mutex_;

  std::vector<unsigned char> costmap_data_;
  unsigned int width_ = 0;
  unsigned int height_ = 0;
  double resolution_ = 0.05;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  std::string global_frame_ = "map";

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
  std::string unified_costmap_topic_;

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
  unsigned int last_start_idx_ = std::numeric_limits<unsigned int>::max();

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr unified_costmap_sub_;
};

} // namespace nav2_dstar_planner

PLUGINLIB_EXPORT_CLASS(nav2_dstar_planner::DStarPlanner, nav2_core::GlobalPlanner)