#include "rclcpp/rclcpp.hpp"
#include "px4_control_interface/srv/plan_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PathGenerator : public rclcpp::Node
{
public:
  PathGenerator()
  : Node("path_generator")
  {
    generate_trajectory_service_ = create_service<px4_control_interface::srv::PlanPath>(
      "generate_trajectory",
      std::bind(&PathGenerator::generateTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));
      
      trajectory_publisher_ = create_publisher<nav_msgs::msg::Path>("trajectory", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  
  }

private:
  void generateTrajectoryCallback(const std::shared_ptr<px4_control_interface::srv::PlanPath::Request> request,
                                  std::shared_ptr<px4_control_interface::srv::PlanPath::Response> response)
  {
    nav_msgs::msg::Path trajectory;
    if(request->type == px4_control_interface::srv::PlanPath::Request::SPIRAL_TYPE) {

        int num_points = request->num_points;
        double radius = request->radius;
        double height = request->height;

        // Customize the trajectory generation based on the request data
        
        trajectory.header.frame_id = "map";

        for (int i = 0; i < num_points; ++i) {
            double angle = (2 * M_PI * i) / num_points;
            double x = radius * std::cos(angle);
            double y = radius * std::sin(angle);
            double z = 2.0 + height * (i / num_points);

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;

            trajectory.poses.push_back(pose);
        }

    } else if(request->type == px4_control_interface::srv::PlanPath::Request::WAYPOINT_TYPE) {

            double target_x = request->target_x;
        double target_y = request->target_y;
        double target_z = request->target_z;

        // Customize the path generation based on the request data
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = target_x;
        pose.pose.position.y = target_y;
        pose.pose.position.z = target_z;

        path.poses.push_back(pose);

        trajectory_publisher_->publish(path);
        response->success = true;
    }
    else
    {
        response->success = false;
    }
    response->trajectory = trajectory;
  }

    rclcpp::Service<px4_control_interface::srv::PlanPath>::SharedPtr generate_trajectory_service_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_publisher_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
