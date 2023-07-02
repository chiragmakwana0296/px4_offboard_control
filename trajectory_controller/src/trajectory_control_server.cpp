#include "trajectory_controller/trajectory_control_server.hpp"
#include <chrono>



TrajectoryController::TrajectoryController() : Node("trajectory_control_server_node")
{
    //   timer_ = this->create_wall_timer(50ms, std::bind(&TrajectoryController::controlCallback, this));
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos, std::bind(&TrajectoryController::odomCallback, this, std::placeholders::_1));

    this->declare_parameter<double>("linear_gain", 0.50);
    this->declare_parameter<double>("angular_gain", 0.50);
    this->declare_parameter<double>("vertical_gain", 0.50);
    this->declare_parameter<double>("controller_rate", 10.0);
    this->declare_parameter<double>("goal_reach_tol", 0.5);
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
    auto path = goal_handle->get_goal();

    if(path->path.poses.size() == 0){
      RCLCPP_INFO(this->get_logger(), "Path Empty ! canceling action call ");
      goal_handle->canceled(result);
    }

    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto trajectory_ = path->path.poses;
    auto path_ned = nav_msgs::msg::Path();
    path_ned.poses.clear();
    for(auto pose_ : trajectory_){
        Eigen::Vector3d pose_enu;
        pose_enu << pose_.pose.position.x, 
                            pose_.pose.position.y, 
                            pose_.pose.position.z;
        Eigen::Vector3d pose_ned = px4_ros_com::frame_transforms::
                                        transform_static_frame(pose_enu, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);

        auto pose = geometry_msgs::msg::PoseStamped();
        pose.pose.position.x = pose_ned.x();
        pose.pose.position.y = pose_ned.y();
        pose.pose.position.z = pose_ned.z();
        path_ned.poses.push_back(pose);
    }

    while (rclcpp::ok())
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
        
        if (current_pose_ned_ != nullptr)
        {
          std::pair<geometry_msgs::msg::Point, double> target_wp = findTargetPoint(*current_pose_ned_, path_ned);
          
          auto target_point = target_wp.first;
          double distance_remaining = calculateDistance(current_pose_ned_->position, path_ned.poses.back().pose.position); //target_wp.second;

          double lookahead_curvature = calculateLookaheadCurvature(target_point, current_pose_ned_->position);

          // angular_velocity_ = linear_velocity_*lookahead_curvature;

          double steering_angle = calculateSteeringAngle(current_pose_ned_->position, current_heading_ned_, target_point);

          // geometry_msgs::msg::Twist cmd_vel = calculateControlCommand(*current_pose_ned_, target_point);

          if(true){
            RCLCPP_INFO_STREAM(this->get_logger(), "steering_angle " << steering_angle);
            RCLCPP_INFO_STREAM(this->get_logger(), "distance_remaining " << distance_remaining);
            RCLCPP_INFO_STREAM(this->get_logger(), "angular_velocity_ " << lookahead_curvature << " linear_velocity_ " << linear_velocity_);
            RCLCPP_INFO_STREAM(this->get_logger(), "lookahead_curvature " << lookahead_curvature);
            RCLCPP_INFO_STREAM(this->get_logger(), "target_point " << target_point.x << " " << target_point.y << " " << target_point.z);
          }

          publish_offboard_control_mode();
          auto trajectory_setpoint_msg = std::make_unique<px4_msgs::msg::TrajectorySetpoint>();
          trajectory_setpoint_msg->position[0] = target_point.x;
          trajectory_setpoint_msg->position[1] = target_point.y;
          trajectory_setpoint_msg->position[2] = target_point.z;
          // trajectory_setpoint_msg->velocity[0] = cmd_vel.linear.x;
          // trajectory_setpoint_msg->velocity[1] = cmd_vel.linear.y;
          // trajectory_setpoint_msg->velocity[2] = cmd_vel.linear.z;
          // trajectory_setpoint_msg->acceleration = ;
          // trajectory_setpoint_msg->yaw = steering_angle; // [-PI:PI]
          // trajectory_setpoint_msg->yawspeed = angular_velocity_; // [-PI:PI]
          trajectory_setpoint_msg->timestamp = this->get_clock()->now().nanoseconds() / 1000;
          trajectory_setpoint_publisher_->publish(std::move(trajectory_setpoint_msg));

          feedback->distance_to_goal = distance_remaining;
          goal_handle->publish_feedback(feedback);

          if(distance_remaining <= 2.0){
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal reached !");
            return;
          }
        }

        if (goal_handle->is_canceling())
        {
          RCLCPP_INFO(this->get_logger(), "Goal canceled !");
          goal_handle->canceled(result);
          return;
        }

        rate.sleep();
    }

}

geometry_msgs::msg::Twist TrajectoryController::calculateControlCommand(const geometry_msgs::msg::Pose &current_pose,
                                                    const geometry_msgs::msg::Point &target_point) {
    // Calculate the desired linear velocity
    double linear_velocity = 1.0;  // Set the desired linear velocity in m/s

    // Calculate the desired velocity vector
    Eigen::Vector3d desired_velocity(target_point.x - current_pose.position.x,
                                     target_point.y - current_pose.position.y,
                                     target_point.z - current_pose.position.z);
    desired_velocity.normalize();
    desired_velocity *= linear_velocity;

    // Calculate the desired position error
    Eigen::Vector3d position_error(target_point.x - current_pose.position.x,
                                   target_point.y - current_pose.position.y,
                                   target_point.z - current_pose.position.z);

    // Calculate the desired velocity error
    Eigen::Vector3d velocity_error(desired_velocity.x() - current_pose.orientation.x,
                                   desired_velocity.y() - current_pose.orientation.y,
                                   desired_velocity.z() - current_pose.orientation.z);

    // Calculate the desired control command
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = desired_velocity.x() + position_error.x() + velocity_error.x();
    cmd_vel.linear.y = desired_velocity.y() + position_error.y() + velocity_error.y();
    cmd_vel.linear.z = desired_velocity.z() + position_error.z() + velocity_error.z();

    return cmd_vel;
  }


double TrajectoryController::calculateSteeringAngle(const geometry_msgs::msg::Point &current_pose, double current_heading,
                                const geometry_msgs::msg::Point &target_point) {
    // Calculate the heading angle in ned
    double target_heading_angle = std::atan2(target_point.y - current_pose.y,
                                 target_point.x - current_pose.x);

    // Calculate the angle difference
    double angle_difference = target_heading_angle - current_heading; // tf2::getYaw(current_pose.orientation);

    // Normalize the angle difference to the range [-pi, pi]
    angle_difference = std::atan2(std::sin(angle_difference), std::cos(angle_difference));

    // Calculate the steering angle
    double steering_angle = 2.0 * angle_difference;

    return steering_angle;
  }

double TrajectoryController::calculateTrajectoryLength(const nav_msgs::msg::Path& path) {
  double length = 0.0;
  for (size_t i = 0; i < path.poses.size() - 1; ++i) {
    const geometry_msgs::msg::Point& p1 = path.poses[i].pose.position;
    const geometry_msgs::msg::Point& p2 = path.poses[i + 1].pose.position;
    double segment_length = calculateDistance(p1, p2);
    length += segment_length;
  }
  return length;
}

std::pair<geometry_msgs::msg::Point, double> TrajectoryController::findTargetPoint(const geometry_msgs::msg::Pose &current_pose,
                                            const nav_msgs::msg::Path &path) {
    double min_distance = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point target_point;
    geometry_msgs::msg::Point curr_target_point;

    size_t index = 0;
    // Find the closest point to the current pose
    for (size_t i = 0; i < path.poses.size(); ++i) {
      double distance = calculateDistance(current_pose.position, path.poses[i].pose.position);

      if (distance < min_distance) {
        min_distance = distance;
        target_point = path.poses[i].pose.position;
        index = i;
      }
    }

    // Calculate the lookahead distance
    double lookahead_distance = std::max(lookahead_distance_, min_distance);
    double distance_traveled{0.0};
    // Find the target point within the lookahead distance
    for (size_t i = index; i < path.poses.size(); ++i) {
      double distance = calculateDistance(current_pose.position, path.poses[i].pose.position);
      distance_traveled += distance;
      if (distance >= lookahead_distance) {
        target_point = path.poses[i+1].pose.position;
        index = i;
        break;
      }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "current index: " << index);

    double distance_remaining = calculateTrajectoryLength(path) - distance_traveled;
    std::pair<geometry_msgs::msg::Point, double> out;
    out.first = target_point;
    out.second = distance_remaining;
    // if(index == path.poses.size()-2){
    // } else{
    //   out.second = false;
    // }
    return out;

  }

double TrajectoryController::calculateLookaheadCurvature(const geometry_msgs::msg::Point &target_point, const geometry_msgs::msg::Point &current_point){
    double distance = calculateDistance(target_point, current_point);
    double dist2 = distance*distance;
    if (dist2 > 0.001) {
        return 2.0 * target_point.x / dist2;
    } else {
        return 0.0;
    }
}

  double TrajectoryController::calculateDistance(const geometry_msgs::msg::Point &point1, const geometry_msgs::msg::Point &point2) {
    double dx = point2.x - point1.x;
    double dy = point2.y - point1.y;
    double dz = point2.z - point1.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}


void TrajectoryController::odomCallback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg){
    if (!current_pose_ned_) {
        current_pose_ned_ = std::make_shared<geometry_msgs::msg::Pose>();
    }
    if (!current_vel_ned_) {
        current_vel_ned_ = std::make_shared<geometry_msgs::msg::Twist>();
    }    
    current_pose_ned_->position.x = msg->x;
    current_pose_ned_->position.y = msg->y;
    current_pose_ned_->position.z = msg->z;
    current_vel_ned_->linear.x = msg->vx;
    current_vel_ned_->linear.y = msg->vy;
    current_vel_ned_->linear.z = msg->vz;
    current_heading_ned_ = msg->heading;

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