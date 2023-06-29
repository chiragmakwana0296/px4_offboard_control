#include "trajectory_controller/trajectory_control_server.hpp"
#include <chrono>



TrajectoryController::TrajectoryController() : Node("trajectory_control_server_node")
{
    //   timer_ = this->create_wall_timer(50ms, std::bind(&TrajectoryController::controlCallback, this));
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", 10, std::bind(&TrajectoryController::odomCallback, this, std::placeholders::_1));


    this->declare_parameter<double>("linear_gain", 0.50);
    this->declare_parameter<double>("angular_gain", 0.50);
    this->declare_parameter<double>("vertical_gain", 0.50);
    this->declare_parameter<double>("controller_rate", 10.0);
    this->declare_parameter<double>("goal_reach_tol", 0.01);
    this->declare_parameter<double>("max_linear_velocity", 1.0);
    this->declare_parameter<double>("max_angular_velocity", 2.0);
    this->declare_parameter<double>("max_linear_acceleration", 0.1);
    this->declare_parameter<double>("max_linear_deceleration", 0.1);
    this->declare_parameter<bool>("add_acce_deccl_limits", true);

    this->get_parameter("linear_gain", linear_gain_);
    this->get_parameter("angular_gain", angular_gain_);
    this->get_parameter("vertical_gain", vertical_gain_);

    this->get_parameter("controller_rate", controller_rate_);
    this->get_parameter("goal_reach_tol", goal_reach_tol_);
    this->get_parameter("max_linear_velocity", max_linear_velocity_);
    this->get_parameter("max_angular_velocity", max_angular_velocity_);
    this->get_parameter("max_linear_acceleration", max_linear_acceleration_);
    this->get_parameter("max_linear_deceleration", max_linear_deceleration_);
    this->get_parameter("add_acce_deccl_limits", add_acce_deccl_limits_);
    
    
    action_server_ = rclcpp_action::create_server<FollowPath>(
      this,
      "path",
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
    rclcpp::Rate rate(controller_rate_);  // 10 Hz

    auto result = std::make_shared<FollowPath::Result>();
    auto feedback = std::make_shared<FollowPath::Feedback>();
    // auto path = std::make_shared<FollowPath::Goal>();

    auto path = goal_handle->get_goal();
    auto trajectory_ = path->path.poses;
    size_t current_trajectory_index = 0;

    while (rclcpp::ok() && current_trajectory_index < trajectory_.size())
    {   

        this->get_parameter("linear_gain", linear_gain_);
        this->get_parameter("angular_gain", angular_gain_);
        this->get_parameter("vertical_gain", vertical_gain_);

        this->get_parameter("goal_reach_tol", goal_reach_tol_);
        this->get_parameter("max_linear_velocity", max_linear_velocity_);
        this->get_parameter("max_angular_velocity", max_angular_velocity_);
        this->get_parameter("max_linear_acceleration", max_linear_acceleration_);
        this->get_parameter("max_linear_deceleration", max_linear_deceleration_);
        this->get_parameter("add_acce_deccl_limits", add_acce_deccl_limits_);

        const geometry_msgs::msg::PoseStamped& current_trajectory_point = trajectory_[current_trajectory_index];
        Eigen::Quaterniond q_ned;
        q_ned.coeffs() << odometry_ned_->q[0], odometry_ned_->q[1],odometry_ned_->q[2],odometry_ned_->q[3];

        Eigen::Vector3d current_goal_enu;
        current_goal_enu << current_trajectory_point.pose.position.x, 
                        current_trajectory_point.pose.position.y, 
                        current_trajectory_point.pose.position.z;

        Eigen::Vector3d current_goal_ned = px4_ros_com::frame_transforms::
                                        transform_static_frame(current_goal_enu, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);

        double x_error = current_goal_ned.x() - odometry_ned_->position[0];
        double y_error = current_goal_ned.y() - odometry_ned_->position[1];
        double z_error = current_goal_ned.z() - odometry_ned_->position[2];
        
        double distance_to_point = std::hypot(x_error, y_error, z_error);

        if (distance_to_point <= goal_reach_tol_)
        {
            current_trajectory_index++;
            continue;
        }

        

        // px4_ros_com::frame_transforms::
        //                 utils::quaternion::transform_orientation(q_ned, StaticTF::NED_TO_ENU);
        
        current_yaw_ = px4_ros_com::
                        frame_transforms::
                        utils::quaternion::quaternion_get_yaw(q_ned);

        double desired_heading = std::atan2(y_error, x_error);
        
        double yaw_error = desired_heading - current_yaw_;

        double linear_velocity = linear_gain_ * distance_to_point;
        double angular_velocity = angular_gain_ * yaw_error;
        
        // Calculate velocity vector components
        double vx = linear_velocity * std::cos(current_yaw_);
        double vy = linear_velocity * std::sin(current_yaw_);
        double vz = vertical_gain_ * z_error;


        publish_offboard_control_mode();
        auto trajectory_setpoint_msg = std::make_unique<px4_msgs::msg::TrajectorySetpoint>();
        trajectory_setpoint_msg->position[0] = current_goal_ned.x();
        trajectory_setpoint_msg->position[1] = current_goal_ned.y();
        trajectory_setpoint_msg->position[2] = current_goal_ned.z();
        trajectory_setpoint_msg->velocity[0] = vx;
        trajectory_setpoint_msg->velocity[1] = vy;
        trajectory_setpoint_msg->velocity[2] = vz;
        // trajectory_setpoint_msg->acceleration = ;
        trajectory_setpoint_msg->yaw = desired_heading; // [-PI:PI]
        trajectory_setpoint_msg->yawspeed = angular_velocity; // [-PI:PI]
        trajectory_setpoint_msg->timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(std::move(trajectory_setpoint_msg));
        rate.sleep();
    }

}

void TrajectoryController::odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
    odometry_ned_ = msg;

    
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


void TrajectoryController::publish_offboard_control_mode()
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}


void TrajectoryController::publish_trajectory_setpoint()
{
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -5.0};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
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