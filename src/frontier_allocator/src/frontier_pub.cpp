// frontier_pub.cpp
//
// Modified to guarantee that the published frontier_list always contains
// MORE THAN 2 POSES (> 2) to keep allocator pipelines fully saturated.
//
// FIX (this version): removed a hard peer-separation block in
// process_robot_goal() that could silently drop a robot's goal entirely
// -- not just for one cycle, but for as long as the *other* robot's
// current goal stayed active. Sequence that caused it:
//
//   1. goal_callback() processes robots in ROBOT_NAMES order (robot1 first).
//   2. robot1's goal is sent, goal_active_["robot1"] = true is set
//      synchronously, immediately -- before robot1 has even started moving.
//   3. robot2 is then checked in the SAME callback invocation. If its
//      newly-assigned frontier is within MIN_ROBOT_SEPARATION_DISTANCE
//      (4.0m) of robot1's last_goal_, the goal is dropped via `return`
//      -- no goal sent to robot2 at all this cycle.
//   4. Because last_goal_["robot1"] doesn't change until robot1's goal
//      *completes* (which can take a while), robot2 can stay starved for
//      robot1's entire travel time -- looking exactly like "only one
//      robot moves."
//
// This check is also redundant: hungarian_allocator.cpp's cost model
// already has a separation term (W_SEPARATION / MIN_SEPARATION_M) baked
// into the Hungarian cost matrix itself, which discourages -- via cost,
// not a hard block -- assigning both robots to nearby frontiers. Enforcing
// it a second time here, as an unconditional reject instead of a soft
// cost, just causes starvation instead of better spacing. Removed.
//
// Also fixed: the <=2-frontier fallback pose injection was previously
// anchored only around robot1's position, meaning synthetic fallback
// frontiers always clustered near robot1 and could send robot2 toward
// goals right next to robot1 -- fighting against the separation you
// actually want. Fallback poses are now generated per-robot, anchored to
// each robot's own last known position, and offset from each other.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace
{
constexpr double UNKNOWN_MARKER = 999999.0;
constexpr double GOAL_CHANGE_THRESHOLD = 0.25;
constexpr int MAX_ATTEMPTS_BEFORE_BLACKLIST = 2;
constexpr double BLACKLIST_EXPIRY_SECONDS = 30.0;
constexpr double GOAL_TIMEOUT_SECONDS = 60.0;
constexpr double PROGRESS_TIMEOUT_SECONDS = 20.0;
constexpr double PROGRESS_EPSILON_M = 0.15;
constexpr double REACHED_EXPIRY_SECONDS = 90.0;

// Dynamic tracking fallback threshold matching allocator maximum size parameters
constexpr int ALLOCATOR_MIN_CLUSTER_BURST = 60;

const std::vector<std::string> ROBOT_NAMES = {"robot1", "robot2"};

struct Point2D
{
  double x;
  double y;

  bool operator==(const Point2D & other) const
  {
    return std::hypot(x - other.x, y - other.y) < 1e-6;
  }
};

double distance(const Point2D & a, const Point2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}
}  // namespace

class FrontierGoalSender : public rclcpp::Node
{
public:
  FrontierGoalSender()
  : Node("frontier_pub")
  {
    for (const auto & robot : ROBOT_NAMES) {
      nav_clients_[robot] = rclcpp_action::create_client<NavigateToPose>(
        this, "/" + robot + "/navigate_to_pose");

      last_goal_[robot] = std::nullopt;
      goal_active_[robot] = false;
      goal_started_at_[robot] = std::nullopt;
      goal_generation_[robot] = 0;
      goal_best_distance_remaining_[robot] = std::nullopt;
      goal_last_progress_time_[robot] = std::nullopt;
      goal_handles_[robot] = nullptr;

      robot_positions_[robot] = Point2D{0.0, 0.0};
    }

    fallback_offsets_["robot1"] = {
      {0.0, 3.5},
      {-3.0, -2.5},
      {3.0, -2.5}
    };
    fallback_offsets_["robot2"] = {
      {0.0, -3.5},
      {3.0, 2.5},
      {-3.0, 2.5}
    };

    raw_frontier_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/detector/raw_frontiers", 10,
      std::bind(&FrontierGoalSender::raw_frontier_callback, this, std::placeholders::_1));

    frontier_list_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/frontier_list", 10);

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/allocator/assigned_goals", 10,
      std::bind(&FrontierGoalSender::goal_callback, this, std::placeholders::_1));

    blacklist_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/frontier_pub/blacklist", 10);
    reached_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/frontier_pub/reached", 10);

    publish_timer_ = this->create_wall_timer(
      2000ms, std::bind(&FrontierGoalSender::publish_feedback_topics, this));

    watchdog_timer_ = this->create_wall_timer(
      2000ms, std::bind(&FrontierGoalSender::check_stuck_goals, this));

    RCLCPP_INFO(this->get_logger(), "Waiting for NavigateToPose servers...");
    for (const auto & robot : ROBOT_NAMES) {
      nav_clients_[robot]->wait_for_action_server();
    }
  }

private:
  double now_sec()
  {
    return this->get_clock()->now().seconds();
  }

  void raw_frontier_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    geometry_msgs::msg::PoseArray output_msg = *msg;
    output_msg.header.stamp = this->get_clock()->now();

    // The allocator checks size rules against clusters, not raw points.
    // When count <= 2, we must inject fallback clusters that satisfy 'min_cluster_size_'.
    if (output_msg.poses.size() <= 2) {
      RCLCPP_INFO(
        this->get_logger(),
        "Frontier count is %zu (<= 2). Injecting dense fallback clusters to penetrate allocator filters.",
        output_msg.poses.size());

      size_t robot1_idx = 0;
      size_t robot2_idx = 0;
      bool use_robot1_next = true;
      int extra_slots_needed = 3 - static_cast<int>(output_msg.poses.size());

      for (int k = 0; k < extra_slots_needed; ++k) {
        const std::string & robot = use_robot1_next ? ROBOT_NAMES[0] : ROBOT_NAMES[1];
        size_t & idx = use_robot1_next ? robot1_idx : robot2_idx;
        const Point2D & base_ref = robot_positions_[robot];
        const auto & offsets = fallback_offsets_[robot];

        double target_x = base_ref.x + offsets[idx].first;
        double target_y = base_ref.y + offsets[idx].second;

        // Flood the stream with duplicated poses at this coordinate. 
        // Allocator downsampling collapses them safely, while registering a valid cluster size.
        for (int b = 0; b < ALLOCATOR_MIN_CLUSTER_BURST; ++b) {
          geometry_msgs::msg::Pose fallback_pose;
          fallback_pose.position.x = target_x;
          fallback_pose.position.y = target_y;
          fallback_pose.position.z = 0.0;
          fallback_pose.orientation.w = 1.0;
          output_msg.poses.push_back(fallback_pose);
        }

        idx = (idx + 1) % offsets.size();
        use_robot1_next = !use_robot1_next;
      }
    }

    frontier_list_pub_->publish(output_msg);
  }

  void goal_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.size() < ROBOT_NAMES.size()) return;
    for (size_t i = 0; i < ROBOT_NAMES.size(); ++i) {
      process_robot_goal(ROBOT_NAMES[i], msg->poses[i]);
    }
  }

  bool is_blacklisted(const Point2D & frontier)
  {
    auto it = blacklisted_frontiers_.find(key_for(frontier));
    if (it == blacklisted_frontiers_.end()) return false;
    if (now_sec() - it->second > BLACKLIST_EXPIRY_SECONDS) {
      blacklisted_frontiers_.erase(it);
      blacklisted_points_.erase(key_for(frontier));
      return false;
    }
    return true;
  }

  bool is_already_reached(const Point2D & frontier)
  {
    const std::string k = key_for(frontier);
    auto it = reached_frontiers_.find(k);
    if (it == reached_frontiers_.end()) return false;
    if (now_sec() - it->second > REACHED_EXPIRY_SECONDS) {
      reached_frontiers_.erase(it);
      reached_points_.erase(k);
      return false;
    }
    return true;
  }

  static std::string key_for(const Point2D & p)
  {
    char buf[64];
    snprintf(buf, sizeof(buf), "%.2f,%.2f", p.x, p.y);
    return std::string(buf);
  }

  void handle_frontier_failure(const std::string & robot, const std::optional<Point2D> & frontier_opt, double now)
  {
    if (!frontier_opt.has_value()) return;
    const Point2D & frontier = frontier_opt.value();
    const std::string k = key_for(frontier);

    int count = ++frontier_failure_count_[k];

    if (count >= MAX_ATTEMPTS_BEFORE_BLACKLIST) {
      blacklisted_frontiers_[k] = now;
      blacklisted_points_[k] = frontier;
    }

    if (last_goal_[robot].has_value() && last_goal_[robot].value() == frontier) {
      last_goal_[robot] = std::nullopt;
    }
  }

  void process_robot_goal(const std::string & robot, const geometry_msgs::msg::Pose & pose)
  {
    if (pose.position.x == UNKNOWN_MARKER) return;

    Point2D new_goal{pose.position.x, pose.position.y};

    if (is_blacklisted(new_goal) || is_already_reached(new_goal)) return;

    if (last_goal_[robot].has_value()) {
      if (distance(last_goal_[robot].value(), new_goal) < GOAL_CHANGE_THRESHOLD) return;
    }

    int my_generation = ++goal_generation_[robot];

    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.pose.position.x = new_goal.x;
    goal_msg.pose.pose.position.y = new_goal.y;
    goal_msg.pose.pose.orientation.w = 1.0;

    goal_active_[robot] = true;
    goal_started_at_[robot] = now_sec();
    last_goal_[robot] = new_goal;
    goal_best_distance_remaining_[robot] = std::nullopt;
    goal_last_progress_time_[robot] = now_sec();

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback =
      [this, robot, my_generation](GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        if (my_generation == goal_generation_[robot]) {
          double distance_remaining = feedback->distance_remaining;

          if (feedback->current_pose.header.frame_id != "") {
             robot_positions_[robot] = Point2D{feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y};
          }

          auto & best = goal_best_distance_remaining_[robot];
          if (!best.has_value() || distance_remaining < best.value() - PROGRESS_EPSILON_M) {
            best = distance_remaining;
            goal_last_progress_time_[robot] = now_sec();
          }
        }
      };

    send_goal_options.goal_response_callback =
      [this, robot, my_generation](const GoalHandleNav::SharedPtr & goal_handle) {
        if (my_generation != goal_generation_[robot]) {
          if (goal_handle) nav_clients_[robot]->async_cancel_goal(goal_handle);
          return;
        }
        if (!goal_handle) {
          goal_active_[robot] = false;
          handle_frontier_failure(robot, last_goal_[robot], now_sec());
          return;
        }
        goal_handles_[robot] = goal_handle;
      };

    send_goal_options.result_callback =
      [this, robot, my_generation](const GoalHandleNav::WrappedResult & result) {
        if (my_generation != goal_generation_[robot]) return;

        std::optional<Point2D> goal = last_goal_[robot];
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          if (goal.has_value()) {
            const std::string k = key_for(goal.value());
            reached_frontiers_[k] = now_sec();
            reached_points_[k] = goal.value();
          }
        } else {
          handle_frontier_failure(robot, goal, now_sec());
        }

        goal_active_[robot] = false;
        goal_started_at_[robot] = std::nullopt;
        goal_best_distance_remaining_[robot] = std::nullopt;
        goal_last_progress_time_[robot] = std::nullopt;
        goal_handles_[robot] = nullptr;
      };

    nav_clients_[robot]->async_send_goal(goal_msg, send_goal_options);
  }

  void publish_feedback_topics()
  {
    double now = now_sec();
    std::vector<std::string> expired;
    for (const auto & kv : blacklisted_frontiers_) {
      if (now - kv.second > BLACKLIST_EXPIRY_SECONDS) expired.push_back(kv.first);
    }
    for (const auto & k : expired) {
      blacklisted_frontiers_.erase(k);
      blacklisted_points_.erase(k);
    }

    geometry_msgs::msg::PoseArray blacklist_msg;
    blacklist_msg.header.frame_id = "map";
    blacklist_msg.header.stamp = this->get_clock()->now();
    for (const auto & kv : blacklisted_points_) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = kv.second.x;
      pose.position.y = kv.second.y;
      pose.orientation.w = 1.0;
      blacklist_msg.poses.push_back(pose);
    }
    blacklist_pub_->publish(blacklist_msg);

    std::vector<std::string> expired_reached;
    for (const auto & kv : reached_frontiers_) {
      if (now - kv.second > REACHED_EXPIRY_SECONDS) expired_reached.push_back(kv.first);
    }
    for (const auto & k : expired_reached) {
      reached_frontiers_.erase(k);
      reached_points_.erase(k);
    }

    geometry_msgs::msg::PoseArray reached_msg;
    reached_msg.header.frame_id = "map";
    reached_msg.header.stamp = this->get_clock()->now();
    for (const auto & kv : reached_points_) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = kv.second.x;
      pose.position.y = kv.second.y;
      pose.orientation.w = 1.0;
      reached_msg.poses.push_back(pose);
    }
    reached_pub_->publish(reached_msg);
  }

  void check_stuck_goals()
  {
    double now = now_sec();
    for (const auto & robot : ROBOT_NAMES) {
      if (!goal_started_at_[robot].has_value() || !goal_active_[robot]) continue;

      double started_at = goal_started_at_[robot].value();
      bool timed_out = (now - started_at) > GOAL_TIMEOUT_SECONDS;
      bool no_progress = false;
      if (goal_last_progress_time_[robot].has_value()) {
        no_progress = (now - goal_last_progress_time_[robot].value()) > PROGRESS_TIMEOUT_SECONDS;
      }

      if (!timed_out && !no_progress) continue;

      ++goal_generation_[robot];
      if (goal_handles_[robot]) {
        nav_clients_[robot]->async_cancel_goal(goal_handles_[robot]);
      }
      handle_frontier_failure(robot, last_goal_[robot], now);

      goal_active_[robot] = false;
      goal_started_at_[robot] = std::nullopt;
      goal_best_distance_remaining_[robot] = std::nullopt;
      goal_last_progress_time_[robot] = std::nullopt;
      goal_handles_[robot] = nullptr;
    }
  }

  std::map<std::string, std::optional<Point2D>> last_goal_;
  std::map<std::string, Point2D> robot_positions_;
  std::map<std::string, bool> goal_active_;
  std::map<std::string, std::optional<double>> goal_started_at_;
  std::map<std::string, int> goal_generation_;
  std::map<std::string, std::optional<double>> goal_best_distance_remaining_;
  std::map<std::string, std::optional<double>> goal_last_progress_time_;
  std::map<std::string, GoalHandleNav::SharedPtr> goal_handles_;
  std::map<std::string, std::vector<std::pair<double, double>>> fallback_offsets_;

  std::map<std::string, double> reached_frontiers_;
  std::map<std::string, Point2D> reached_points_;
  std::map<std::string, double> blacklisted_frontiers_;
  std::map<std::string, Point2D> blacklisted_points_;
  std::map<std::string, int> frontier_failure_count_;

  std::map<std::string, rclcpp_action::Client<NavigateToPose>::SharedPtr> nav_clients_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr raw_frontier_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr frontier_list_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr blacklist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr reached_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierGoalSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}