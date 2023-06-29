#ifndef __TRAJECTORY_CONTROLLER_SERVER_HPP__
#define __TRAJECTORY_CONTROLLER_SERVER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
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

    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    void arm();
    void disarm();
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;

    // rclcpp::TimerBase::SharedPtr timer_;

    double linear_gain_{0.0};
    double angular_gain_{0.0};
    double vertical_gain_{0.0};
    double controller_rate_{10.0};
    double goal_reach_tol_{0.0};
    double max_linear_velocity_{0.0};
    double max_angular_velocity_{0.0};
    double max_linear_acceleration_{0.0};
    double max_linear_deceleration_{0.0};
    bool add_acce_deccl_limits_{false};

    double current_yaw_{0.0};
    px4_msgs::msg::VehicleOdometry::SharedPtr odometry_ned_; 
};

#endif  // __TRAJECTORY_CONTROLLER_SERVER_HPP__
