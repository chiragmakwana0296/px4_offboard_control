#include "trajectory_controller/commander_action.hpp"
#include <chrono>



TrajectoryController::TrajectoryController() : Node("trajectory_control_server_node")
{
    //   timer_ = this->create_wall_timer(50ms, std::bind(&TrajectoryController::controlCallback, this));
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
    status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", qos, std::bind(&TrajectoryController::statusCallback, this, std::placeholders::_1));
  
    action_server_ = rclcpp_action::create_server<FollowPath>(
      this,
      "commander",
      std::bind(&TrajectoryController::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryController::handleCancel, this, std::placeholders::_1),
      std::bind(&TrajectoryController::handleAccepted, this, std::placeholders::_1));

}


rclcpp_action::GoalResponse TrajectoryController::handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowPath::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
  
rclcpp_action::CancelResponse TrajectoryController::handleCancel(const std::shared_ptr<GoalHandleFollowPath> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received goal cancel request");
    (void)goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
}


void TrajectoryController::handleAccepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle){
    using namespace std::placeholders;
    std::thread{std::bind(&TrajectoryController::execute, this, _1), goal_handle}.detach();
}


void TrajectoryController::execute(const std::shared_ptr<GoalHandleFollowPath> goal_handle){
    

    auto result = std::make_shared<FollowPath::Result>();
    auto feedback = std::make_shared<FollowPath::Feedback>();
    auto command = goal_handle->get_goal();

    rclcpp::Rate rate(command->rate);  // 10 Hz
    int retry{0};
    while (rclcpp::ok())
    {   
        
        if (status_ != nullptr)
        {
          
          switch (command->command) {
              case "ARMING":
                  RCLCPP_INFO(this->get_logger(), "ARMING");
                  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, command->param1, command->param2);
                  feedback->arming_state = status_->arming_state;
                  feedback->mode_state = status_->nav_state
                  goal_handle->publish_feedback(feedback);  

                  if(status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
                    result->success = true;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Armed !");
                    return;
                  } else if (retry >= command->retry ){
                      result->success = false;
                      goal_handle->succeed(result);
                  }
                  break;
              case "SET_MODE":
                  RCLCPP_INFO(this->get_logger(), "SET MODE !");
                  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, command->param1, command->param2);
                  feedback->arming_state = status_->arming_state;
                  feedback->mode_state = status_->nav_state
                  feedback->arming_state = status_->arming_state;
                  goal_handle->publish_feedback(feedback);  

                  if(status_->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD){
                    result->success = true;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Armed !");
                    return;
                  } else if (retry >= command->retry ){
                      result->success = false;
                      goal_handle->succeed(result);
                  }
                  break;
              default:
                  RCLCPP_INFO(this->get_logger(), "Invalid command !");
          }
          
        }

        if (goal_handle->is_canceling())
        {
          RCLCPP_INFO(this->get_logger(), "Goal canceled !");
          goal_handle->canceled(result);
          return;
        }
        retry += 1;
        rate.sleep();
    }

}
void TrajectoryController::statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  status_ = msg;

}

void TrajectoryController::arm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}


void TrajectoryController::disarm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


void TrajectoryController::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto pid_controller = std::make_shared<TrajectoryController>();
  rclcpp::spin(pid_controller);
  rclcpp::shutdown();
  return 0;
}