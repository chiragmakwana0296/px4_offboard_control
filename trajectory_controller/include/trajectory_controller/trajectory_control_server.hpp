#ifndef __TRAJECTORY_CONTROLLER_SERVER_HPP__
#define __TRAJECTORY_CONTROLLER_SERVER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include "px4_ros_com/frame_transforms.h"
#include <stdint.h>
#include <chrono>
#include <iostream>

using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;
using namespace std::chrono;
using namespace std::chrono_literals;

class TrajectoryController : public rclcpp::Node
{
public:
    explicit TrajectoryController();

private:
    rclcpp_action::GoalResponse handleGoal(
                                const rclcpp_action::GoalUUID& uuid,
                                std::shared_ptr<const FollowPath::Goal> goal);
  
    rclcpp_action::CancelResponse handleCancel(
                                const std::shared_ptr<GoalHandleFollowPath> goal_handle);
  
    void execute(const std::shared_ptr<GoalHandleFollowPath> goal_handle);
    void handleAccepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle);

    double compute_distance_remaining(const geometry_msgs::msg::PoseStamped&);
    void controlCallback();

    void odomCallback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg);

    void arm();
    void disarm();
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    std::pair<geometry_msgs::msg::Point, double> findTargetPoint(const geometry_msgs::msg::Pose &current_pose,
                                            const nav_msgs::msg::Path &path);
    double calculateDistance(const geometry_msgs::msg::Point &point1, const geometry_msgs::msg::Point &point2);
    double calculateSteeringAngle(const geometry_msgs::msg::Point &current_pose, double current_heading,
                                const geometry_msgs::msg::Point &target_point);
    geometry_msgs::msg::Twist calculateControlCommand(const geometry_msgs::msg::Pose &current_pose,
                                                    const geometry_msgs::msg::Point &target_point);
    double calculateLookaheadCurvature(const geometry_msgs::msg::Point &target_point, const geometry_msgs::msg::Point &current_point);
    double calculateTrajectoryLength(const nav_msgs::msg::Path& path);
    rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr odom_sub_;

    // rclcpp::TimerBase::SharedPtr timer_;

    double linear_gain_{0.0};
    double angular_gain_{0.0};
    double vertical_gain_{0.0};
    double controller_rate_{10.0};
    double goal_reach_tol_{0.50};
    double max_linear_velocity_{0.0};
    double max_angular_velocity_{0.0};
    double max_linear_acceleration_{0.0};
    double max_linear_deceleration_{0.0};
    bool add_acce_deccl_limits_{false};

    double current_heading_ned_{0.0};
    px4_msgs::msg::VehicleLocalPosition::SharedPtr odometry_ned_; 
    geometry_msgs::msg::Pose::SharedPtr current_pose_ned_; 
    geometry_msgs::msg::Twist::SharedPtr current_vel_ned_; 
    double lookahead_distance_{1.0}; 
    double linear_velocity_{1.0};
    double angular_velocity_{0.0};
    
};

#endif  // __TRAJECTORY_CONTROLLER_SERVER_HPP__
