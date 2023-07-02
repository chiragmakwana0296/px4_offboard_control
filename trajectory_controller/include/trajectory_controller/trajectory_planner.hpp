#ifndef PATH_PUBLISHER_HPP_
#define PATH_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_path.hpp"

class PathPlanner : public rclcpp::Node
{
public:
  PathPlanner();

private:
  nav_msgs::msg::Path generate_spiral_trajectory(int num_points, double radius, double height);
  void publish_path();
  void feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr goal_handle);

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr action_client_;
};

#endif  // PATH_PUBLISHER_HPP_
