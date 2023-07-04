#ifndef _COMMANDER_ACTION_HPP
#define _COMMANDER_ACTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

class TrajectoryController : public rclcpp::Node
{
public:
  using Commander = px4_control_interface::action::Commander;
  using GoalHandleCommander = rclcpp_action::ServerGoalHandle<Commander>;

  TrajectoryController();

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const Commander::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleCommander> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleCommander> goal_handle);

  void execute(const std::shared_ptr<GoalHandleCommander> goal_handle);

  void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  void arm();

  void disarm();

  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;

  rclcpp_action::Server<Commander>::SharedPtr action_server_;

  std::shared_ptr<px4_msgs::msg::VehicleStatus> status_;

  const int controller_rate_ = 10;  // Hz
};

#endif  // _COMMANDER_ACTION_HPP
