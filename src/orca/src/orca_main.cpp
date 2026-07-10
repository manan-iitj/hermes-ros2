#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// RVO2 includes
#include "orca/rvo2/RVOSimulator.h"
#include "orca/rvo2/Vector2.h"

using namespace std::chrono_literals;

class OrcaTurtlebotBridge : public rclcpp::Node
{
public:
    OrcaTurtlebotBridge() : Node("orca_turtlebot_bridge_node")
    {
        sim_ = std::make_unique<RVO::RVOSimulator>();
        sim_->setTimeStep(0.05); 

        // Hardened simulation parameters for high-speed, close-quarters execution
        float neighborDist = 1.5f;
        size_t maxNeighbors = 2;
        float timeHorizon = 0.8f;       
        float timeHorizonObst = 0.0f;   // Blind ORCA to walls so it never freezes near obstacles
        float radius = 0.23f;            // Strict safety radius per robot
        float maxSpeed = 1.5f;           // High-speed capability limit

        sim_->setAgentDefaults(neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed);

        robot1_id_ = sim_->addAgent(RVO::Vector2(0.0f, 0.0f));
        robot2_id_ = sim_->addAgent(RVO::Vector2(0.0f, 0.0f));

        sub_r1_nav2_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/robot1/cmd_vel_raw", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { r1_pref_twist_ = *msg; });
        sub_r2_nav2_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/robot2/cmd_vel_raw", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { r2_pref_twist_ = *msg; });

        sub_r1_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot1/odom", 10, std::bind(&OrcaTurtlebotBridge::robot1OdomCb, this, std::placeholders::_1));
        sub_r2_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot2/odom", 10, std::bind(&OrcaTurtlebotBridge::robot2OdomCb, this, std::placeholders::_1));

        pub_r1_safe_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);
        pub_r2_safe_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot2/cmd_vel", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&OrcaTurtlebotBridge::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Hardened Mutex-Evasion ORCA Bridge Active.");
    }

private:
    void robot1OdomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        r1_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
        r1_pos_ = RVO::Vector2(msg->pose.pose.position.x, msg->pose.pose.position.y);
        sim_->setAgentPosition(robot1_id_, r1_pos_);

        r1_current_vel_ = RVO::Vector2(msg->twist.twist.linear.x * std::cos(r1_yaw_), msg->twist.twist.linear.x * std::sin(r1_yaw_));
        sim_->setAgentVelocity(robot1_id_, r1_current_vel_);
    }

    void robot2OdomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        r2_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
        r2_pos_ = RVO::Vector2(msg->pose.pose.position.x, msg->pose.pose.position.y);
        sim_->setAgentPosition(robot2_id_, r2_pos_);

        r2_current_vel_ = RVO::Vector2(msg->twist.twist.linear.x * std::cos(r2_yaw_), msg->twist.twist.linear.x * std::sin(r2_yaw_));
        sim_->setAgentVelocity(robot2_id_, r2_current_vel_);
    }

    void controlLoop()
    {
        float current_distance = RVO::abs(r1_pos_ - r2_pos_);

        // CONDITION 1: ABSOLUTE PASS-THROUGH (Clear Zone)
        // If the robots are safe or moving away, do not alter Nav2's commands at all.
        RVO::Vector2 relative_pos = r2_pos_ - r1_pos_;
        RVO::Vector2 relative_vel = r2_current_vel_ - r1_current_vel_;
        float dot_product = relative_pos.x() * relative_vel.x() + relative_pos.y() * relative_vel.y();

        if (current_distance > 0.80f || dot_product > 0.0f) {
            pub_r1_safe_->publish(r1_pref_twist_);
            pub_r2_safe_->publish(r2_pref_twist_);
            return;
        }

        // --- CONDITION 2: MUTEX EVASION HIJACK (< 0.80m and closing) ---
        // Force preferred vectors to move forward aggressively at max tracking speeds
        float r1_speed = std::max(0.4f, (float)r1_pref_twist_.linear.x);
        float r2_speed = std::max(0.4f, (float)r2_pref_twist_.linear.x);

        RVO::Vector2 r1_pref(r1_speed * std::cos(r1_yaw_), r1_speed * std::sin(r1_yaw_));
        RVO::Vector2 r2_pref(r2_speed * std::cos(r2_yaw_), r2_speed * std::sin(r2_yaw_));

        // Strong Right-Hand Rule cross-bias to force immediate side resolution
        RVO::Vector2 r1_bias(-r1_pref.y() * 0.40f, r1_pref.x() * 0.40f);
        RVO::Vector2 r2_bias(-r2_pref.y() * 0.40f, r2_pref.x() * 0.40f);
        r1_pref += r1_bias;
        r2_pref += r2_bias;

        sim_->setAgentPrefVelocity(robot1_id_, r1_pref);
        sim_->setAgentPrefVelocity(robot2_id_, r2_pref);

        sim_->doStep();

        // Map velocities using the pure non-holonomic vector projection below
        pub_r1_safe_->publish(executePureEvasion(sim_->getAgentVelocity(robot1_id_), r1_yaw_));
        pub_r2_safe_->publish(executePureEvasion(sim_->getAgentVelocity(robot2_id_), r2_yaw_));
    }

    geometry_msgs::msg::Twist executePureEvasion(const RVO::Vector2& safe_global_vel, double current_yaw)
    {
        geometry_msgs::msg::Twist out_twist;

        if (RVO::absSq(safe_global_vel) < 0.001f) {
            out_twist.linear.x = 0.2; // Keep crawling forward to break static friction/stalls
            out_twist.angular.z = 0.5; // Turn out of the deadlock point
            return out_twist;
        }

        // Project the raw ORCA safe velocity directly onto the robot's local x-axis
        double safe_vx = safe_global_vel.x() * std::cos(current_yaw) + safe_global_vel.y() * std::sin(current_yaw);
        
        // Anti-Stall: Force a baseline forward momentum so the robot drives through its turns
        out_twist.linear.x = std::clamp(safe_vx, 0.3, 1.2);

        // Heading targeting
        double desired_heading = std::atan2(safe_global_vel.y(), safe_global_vel.x());
        double heading_error = desired_heading - current_yaw;
        heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error)); 

        // Aggressive steering gain to clear the collision boundary instantly
        double kp_steering = 4.5; 
        out_twist.angular.z = kp_steering * heading_error;
        out_twist.angular.z = std::clamp(out_twist.angular.z, -2.0, 2.0); 

        return out_twist;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_r1_nav2_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_r2_nav2_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_r1_odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_r2_odom_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_r1_safe_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_r2_safe_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<RVO::RVOSimulator> sim_;
    size_t robot1_id_, robot2_id_;
    double r1_yaw_ = 0.0, r2_yaw_ = 0.0;
    RVO::Vector2 r1_pos_, r2_pos_;
    RVO::Vector2 r1_current_vel_, r2_current_vel_;
    geometry_msgs::msg::Twist r1_pref_twist_, r2_pref_twist_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrcaTurtlebotBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}