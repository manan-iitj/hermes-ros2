// frontier_alg.cpp
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <queue>
#include <algorithm>
#include <numeric>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp" 
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

namespace
{
constexpr double UNKNOWN_MARKER = 999999.0;
constexpr double MATCH_TOLERANCE_M = 0.5;
constexpr double GOAL_REACHED_THRESHOLD = 0.6; 
constexpr double DOWNSAMPLE_RESOLUTION_M = 0.2; 

const std::vector<std::string> ROBOT_NAMES = {"robot1", "robot2"};
const std::string ROBOT_BASE_FRAME_SUFFIX = "base_footprint";
const std::string GLOBAL_FRAME = "map";

struct Point2D
{
  double x;
  double y;
};

struct FrontierCluster
{
  Point2D centroid;
  int size;
};

inline double distance(const Point2D & a, const Point2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

bool is_near_any(const Point2D & p, const std::vector<Point2D> & set, double tolerance)
{
  for (const auto & q : set) {
    if (distance(p, q) < tolerance) {
      return true;
    }
  }
  return false;
}

std::vector<Point2D> downsample_points(const std::vector<Point2D> & points, double res)
{
  if (points.empty()) return {};
  std::vector<Point2D> downsampled;
  for (const auto & pt : points) {
    if (!is_near_any(pt, downsampled, res)) {
      downsampled.push_back(pt);
    }
  }
  return downsampled;
}

std::vector<FrontierCluster> cluster_frontiers_agglomerative(const std::vector<Point2D> & points, double max_link_dist)
{
  if (points.empty()) return {};

  const size_t n = points.size();
  std::vector<int> parent(n);
  std::iota(parent.begin(), parent.end(), 0);

  auto find_set = [&](auto & self, int i) -> int {
    if (parent[i] == i) return i;
    return parent[i] = self(self, parent[i]);
  };

  auto union_sets = [&](int i, int j) {
    int root_i = find_set(find_set, i);
    int root_j = find_set(find_set, j);
    if (root_i != root_j) {
      parent[root_i] = root_j;
    }
  };

  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      if (distance(points[i], points[j]) <= max_link_dist) {
        union_sets(i, j);
      }
    }
  }

  std::vector<std::vector<Point2D>> grouped_points(n);
  for (size_t i = 0; i < n; ++i) {
    int root = find_set(find_set, static_cast<int>(i));
    grouped_points[root].push_back(points[i]);
  }

  std::vector<FrontierCluster> clusters;
  for (size_t i = 0; i < n; ++i) {
    if (grouped_points[i].empty()) continue;

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto & pt : grouped_points[i]) {
      sum_x += pt.x;
      sum_y += pt.y;
    }

    FrontierCluster cl;
    cl.centroid.x = sum_x / grouped_points[i].size();
    cl.centroid.y = sum_y / grouped_points[i].size();
    cl.size = static_cast<int>(grouped_points[i].size());
    clusters.push_back(cl);
  }

  return clusters;
}

std::vector<int> solve_hungarian_n_le_m(const std::vector<std::vector<double>> & cost, int n, int m)
{
  const double INF = std::numeric_limits<double>::infinity();
  std::vector<double> u(n + 1, 0.0), v(m + 1, 0.0);
  std::vector<int> p(m + 1, 0), way(m + 1, 0);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(m + 1, INF);
    std::vector<char> used(m + 1, 0);

    do {
      used[j0] = 1;
      int i0 = p[j0];
      int j1 = -1;
      double delta = INF;

      for (int j = 1; j <= m; ++j) {
        if (!used[j]) {
          double cur = cost[i0][j] - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
      }

      for (int j = 0; j <= m; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }

  std::vector<int> job_for_worker(n + 1, -1);
  for (int j = 1; j <= m; ++j) {
    if (p[j] != 0) {
      job_for_worker[p[j]] = j;
    }
  }
  return job_for_worker;
}
}  // namespace

class HungarianAllocatorNode : public rclcpp::Node
{
public:
  HungarianAllocatorNode()
  : Node("frontier_alg"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    robot_positions_.resize(ROBOT_NAMES.size(), Point2D{0.0, 0.0});
    robot_pose_received_.resize(ROBOT_NAMES.size(), false);
    last_assigned_goal_.resize(ROBOT_NAMES.size(), std::nullopt);

    exploration_pct_ = 0.0;
    update_dynamic_parameters();

    frontier_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/frontier_list", 10,
      std::bind(&HungarianAllocatorNode::frontier_callback, this, std::placeholders::_1));

    blacklist_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/frontier_pub/blacklist", 10,
      std::bind(&HungarianAllocatorNode::blacklist_callback, this, std::placeholders::_1));

    reached_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/frontier_pub/reached", 10,
      std::bind(&HungarianAllocatorNode::reached_callback, this, std::placeholders::_1));

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
      std::bind(&HungarianAllocatorNode::map_callback, this, std::placeholders::_1));

    goal_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/allocator/assigned_goals", 10);

    tf_timer_ = this->create_wall_timer(
      200ms, std::bind(&HungarianAllocatorNode::update_robot_poses_from_tf, this));

    allocation_timer_ = this->create_wall_timer(
      1000ms, std::bind(&HungarianAllocatorNode::run_allocation, this));

    RCLCPP_INFO(this->get_logger(), "Robust Hungarian Centroid Allocator running smoothly.");
  }

private:
  void frontier_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    frontiers_.clear();
    for (const auto & pose : msg->poses) {
      frontiers_.push_back(Point2D{pose.position.x, pose.position.y});
    }
  }

  void blacklist_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    blacklisted_frontiers_.clear();
    for (const auto & pose : msg->poses) {
      blacklisted_frontiers_.push_back(Point2D{pose.position.x, pose.position.y});
    }
  }

  void reached_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    reached_frontiers_.clear();
    for (const auto & pose : msg->poses) {
      reached_frontiers_.push_back(Point2D{pose.position.x, pose.position.y});
    }
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (msg->data.empty()) return;

    long long explored_cells = 0;
    long long total_cells = msg->data.size();

    for (const auto & cell : msg->data) {
      if (cell >= 0) {
        explored_cells++;
      }
    }

    if (total_cells > 0) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      exploration_pct_ = static_cast<double>(explored_cells) / static_cast<double>(total_cells);
      update_dynamic_parameters();
    }
  }

  void update_dynamic_parameters()
  {
    double t = std::clamp(exploration_pct_, 0.0, 1.0);

    if (t < 0.90) { 
      w_distance_           = 0.8;    
      w_richness_           = 10.0;   
      w_separation_         = 45.0;   
      min_separation_m_     = 10.0;   
      w_current_goal_bonus_ = 50.0;   
      cluster_radius_m_     = 1.5;    
      min_cluster_size_     = 15;     
    }
    else {
      w_distance_           = 40.0;   
      w_richness_           = 0.0;    
      w_separation_         = 0.0;    
      min_separation_m_     = 0.0;    
      w_current_goal_bonus_ = 10.0;   
      cluster_radius_m_     = 1.0;    
      min_cluster_size_     = 30;     
    }
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
      "Exploration Progress: %.1f%% | Min Cluster Size: %d",
      exploration_pct_ * 100.0, min_cluster_size_);
  }
  
  void update_robot_poses_from_tf()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < ROBOT_NAMES.size(); ++i) {
      const std::string robot_frame = ROBOT_NAMES[i] + "/" + ROBOT_BASE_FRAME_SUFFIX;
      try {
        geometry_msgs::msg::TransformStamped transform =
          tf_buffer_.lookupTransform(GLOBAL_FRAME, robot_frame, tf2::TimePointZero);

        robot_positions_[i].x = transform.transform.translation.x;
        robot_positions_[i].y = transform.transform.translation.y;
        robot_pose_received_[i] = true;

        if (last_assigned_goal_[i].has_value()) {
          if (distance(robot_positions_[i], last_assigned_goal_[i].value()) < GOAL_REACHED_THRESHOLD) {
            last_assigned_goal_[i] = std::nullopt;
          }
        }
      } catch (const tf2::TransformException & ex) {
        // Safe context drop if frames aren't ready yet
      }
    }
  }

  // Expects external locking where called if using shared data
  bool is_excluded_unlocked(const Point2D & f, const std::vector<Point2D>& blacklist, const std::vector<Point2D>& reached) const
  {
    return is_near_any(f, blacklist, MATCH_TOLERANCE_M) ||
           is_near_any(f, reached, MATCH_TOLERANCE_M);
  }

  double compute_cost(
    size_t robot_index_global,
    const Point2D & robot_pos,
    const Point2D & cluster_centroid,
    int cluster_size) const
  {
    double cost = w_distance_ * distance(robot_pos, cluster_centroid);
    cost -= w_richness_ * static_cast<double>(cluster_size);

    if (last_assigned_goal_[robot_index_global].has_value()) {
      if (distance(cluster_centroid, last_assigned_goal_[robot_index_global].value()) < MATCH_TOLERANCE_M) {
        cost -= w_current_goal_bonus_;
      }
    }

    for (size_t other = 0; other < ROBOT_NAMES.size(); ++other) {
      if (other == robot_index_global) continue;
      
      if (last_assigned_goal_[other].has_value()) {
        double sep = distance(cluster_centroid, last_assigned_goal_[other].value());
        if (sep < min_separation_m_) {
          cost += w_separation_ * (min_separation_m_ - sep);
        }
      }
    }
    return cost;
  }

  void run_allocation()
  {
    std::vector<Point2D> raw_frontiers;
    std::vector<Point2D> local_blacklist;
    std::vector<Point2D> local_reached;
    std::vector<size_t> known_robot_indices;

    // STEP 1: Perform localized, rapid deep copies to free the lock instantly
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (frontiers_.empty()) return;
      raw_frontiers = frontiers_;
      local_blacklist = blacklisted_frontiers_;
      local_reached = reached_frontiers_;
      
      for (size_t i = 0; i < ROBOT_NAMES.size(); ++i) {
        if (robot_pose_received_[i]) {
          known_robot_indices.push_back(i);
        }
      }
    }

    if (known_robot_indices.empty() || raw_frontiers.empty()) return;

    // STEP 2: Process heavy algorithmic geometry open-threaded (NO LOCKS HELD HERE)
    std::vector<Point2D> local_frontiers = downsample_points(raw_frontiers, DOWNSAMPLE_RESOLUTION_M);
    if (local_frontiers.empty()) return;

    std::vector<FrontierCluster> raw_clusters = cluster_frontiers_agglomerative(local_frontiers, cluster_radius_m_);
    std::vector<FrontierCluster> candidate_clusters;

    for (const auto & cl : raw_clusters) {
      if (!is_excluded_unlocked(cl.centroid, local_blacklist, local_reached) && cl.size >= min_cluster_size_) {
        candidate_clusters.push_back(cl);
      }
    }

    if (candidate_clusters.empty()) {
      for (const auto & cl : raw_clusters) {
        if (!is_excluded_unlocked(cl.centroid, local_blacklist, local_reached)) {
          candidate_clusters.push_back(cl);
        }
      }
    }

    geometry_msgs::msg::PoseArray output;
    output.header.frame_id = GLOBAL_FRAME;
    output.header.stamp = this->get_clock()->now();

    // STEP 3: Re-lock exclusively for the assignment calculations that write back state
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (candidate_clusters.empty()) {
      for (size_t i = 0; i < ROBOT_NAMES.size(); ++i) {
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        if (last_assigned_goal_[i].has_value() && !is_excluded_unlocked(last_assigned_goal_[i].value(), blacklisted_frontiers_, reached_frontiers_)) {
          pose.position.x = last_assigned_goal_[i]->x;
          pose.position.y = last_assigned_goal_[i]->y;
        } else {
          pose.position.x = UNKNOWN_MARKER;
          pose.position.y = UNKNOWN_MARKER;
          last_assigned_goal_[i] = std::nullopt;
        }
        output.poses.push_back(pose);
      }
      goal_pub->publish(output);
      return;
    }

    std::vector<int> job_for_robot(known_robot_indices.size(), -1);
    std::vector<bool> cluster_claimed(candidate_clusters.size(), false);

    std::vector<size_t> free_robot_row_indices; 
    std::vector<int> free_cluster_col_indices;

    for (size_t row = 0; row < known_robot_indices.size(); ++row) {
      size_t robot_global_idx = known_robot_indices[row];
      bool holds_valid_lock = false;

      if (last_assigned_goal_[robot_global_idx].has_value()) {
        for (size_t col = 0; col < candidate_clusters.size(); ++col) {
          if (!cluster_claimed[col] && 
              distance(candidate_clusters[col].centroid, last_assigned_goal_[robot_global_idx].value()) < MATCH_TOLERANCE_M) 
          {
            job_for_robot[row] = static_cast<int>(col);
            cluster_claimed[col] = true;
            holds_valid_lock = true;
            break;
          }
        }
      }
      if (!holds_valid_lock) {
        free_robot_row_indices.push_back(row);
      }
    }

    for (size_t col = 0; col < candidate_clusters.size(); ++col) {
      if (!cluster_claimed[col]) {
        free_cluster_col_indices.push_back(static_cast<int>(col));
      }
    }

    int num_free_robots = static_cast<int>(free_robot_row_indices.size());
    int num_free_clusters = static_cast<int>(free_cluster_col_indices.size());

    if (num_free_robots > 0 && num_free_clusters > 0) {
      if (num_free_robots <= num_free_clusters) {
        std::vector<std::vector<double>> cost(num_free_robots + 1, std::vector<double>(num_free_clusters + 1, 0.0));
        for (int row = 0; row < num_free_robots; ++row) {
          size_t robot_global = known_robot_indices[free_robot_row_indices[row]];
          const auto & rp = robot_positions_[robot_global];
          for (int col = 0; col < num_free_clusters; ++col) {
            int target_cluster_idx = free_cluster_col_indices[col];
            cost[row + 1][col + 1] =
              compute_cost(robot_global, rp, candidate_clusters[target_cluster_idx].centroid, candidate_clusters[target_cluster_idx].size);
          }
        }
        auto result = solve_hungarian_n_le_m(cost, num_free_robots, num_free_clusters);
        for (int row = 1; row <= num_free_robots; ++row) {
          int free_assignment = result[row] - 1;
          if (free_assignment >= 0) {
            int chosen_cluster = free_cluster_col_indices[free_assignment];
            job_for_robot[free_robot_row_indices[row - 1]] = chosen_cluster;
            cluster_claimed[chosen_cluster] = true;
          }
        }
      } else {
        std::vector<std::vector<double>> cost(num_free_clusters + 1, std::vector<double>(num_free_robots + 1, 0.0));
        for (int row = 0; row < num_free_clusters; ++row) {
          int target_cluster_idx = free_cluster_col_indices[row];
          for (int col = 0; col < num_free_robots; ++col) {
            size_t robot_global = known_robot_indices[free_robot_row_indices[col]];
            const auto & rp = robot_positions_[robot_global];
            cost[row + 1][col + 1] =
              compute_cost(robot_global, rp, candidate_clusters[target_cluster_idx].centroid, candidate_clusters[target_cluster_idx].size);
          }
        }
        auto result = solve_hungarian_n_le_m(cost, num_free_clusters, num_free_robots);
        for (int cluster_row = 1; cluster_row <= num_free_clusters; ++cluster_row) {
          int worker_col = result[cluster_row];
          if (worker_col > 0 && worker_col <= num_free_robots) {
            int chosen_cluster = free_cluster_col_indices[cluster_row - 1];
            job_for_robot[free_robot_row_indices[worker_col - 1]] = chosen_cluster;
            cluster_claimed[chosen_cluster] = true;
          }
        }
      }
    }

    for (size_t robot_index = 0; robot_index < ROBOT_NAMES.size(); ++robot_index) {
      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;

      auto it = std::find(known_robot_indices.begin(), known_robot_indices.end(), robot_index);

      if (it != known_robot_indices.end()) {
        int row = static_cast<int>(std::distance(known_robot_indices.begin(), it));
        int col = job_for_robot[row];

        if (col >= 0 && col < static_cast<int>(candidate_clusters.size())) {
          pose.position.x = candidate_clusters[col].centroid.x;
          pose.position.y = candidate_clusters[col].centroid.y;
          last_assigned_goal_[robot_index] = candidate_clusters[col].centroid;
        } else {
          if (last_assigned_goal_[robot_index].has_value() && !is_excluded_unlocked(last_assigned_goal_[robot_index].value(), blacklisted_frontiers_, reached_frontiers_)) {
            pose.position.x = last_assigned_goal_[robot_index]->x;
            pose.position.y = last_assigned_goal_[robot_index]->y;
          } else {
            int fallback_idx = -1;
            int max_size = -1;
            for (size_t c = 0; c < candidate_clusters.size(); ++c) {
              if (!cluster_claimed[c] && candidate_clusters[c].size > max_size) {
                max_size = candidate_clusters[c].size;
                fallback_idx = static_cast<int>(c);
              }
            }
            if (fallback_idx == -1) { 
              for (size_t c = 0; c < candidate_clusters.size(); ++c) {
                if (candidate_clusters[c].size > max_size) {
                  max_size = candidate_clusters[c].size;
                  fallback_idx = static_cast<int>(c);
                }
              }
            }

            if (fallback_idx != -1) {
              pose.position.x = candidate_clusters[fallback_idx].centroid.x;
              pose.position.y = candidate_clusters[fallback_idx].centroid.y;
              last_assigned_goal_[robot_index] = candidate_clusters[fallback_idx].centroid;
              cluster_claimed[fallback_idx] = true;
            } else {
              pose.position.x = UNKNOWN_MARKER;
              pose.position.y = UNKNOWN_MARKER;
            }
          }
        }
      } else {
        pose.position.x = UNKNOWN_MARKER;
        pose.position.y = UNKNOWN_MARKER;
      }
      output.poses.push_back(pose);
    }
    goal_pub->publish(output);
  }

  std::mutex data_mutex_;
  double exploration_pct_;
  double w_distance_;
  double w_richness_;
  double w_separation_;
  double min_separation_m_;
  int min_cluster_size_;
  double cluster_radius_m_;
  double w_current_goal_bonus_;

  std::vector<Point2D> frontiers_;
  std::vector<Point2D> blacklisted_frontiers_;
  std::vector<Point2D> reached_frontiers_;
  std::vector<Point2D> robot_positions_;
  std::vector<bool> robot_pose_received_;
  std::vector<std::optional<Point2D>> last_assigned_goal_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr frontier_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr blacklist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr reached_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goal_pub;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr allocation_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
}; 

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HungarianAllocatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}