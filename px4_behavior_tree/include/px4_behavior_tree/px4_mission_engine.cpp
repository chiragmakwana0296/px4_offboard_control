#ifndef _PX4_MISSION_ENGINE_
#define _PX4_MISSION_ENGINE_

#include <memory>
#include <string>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/loggers/bt_zmq_publisher.h"

using namespace BT;

class Px4MissionEngine : public rclcpp::Node
{
private:
  BehaviorTreeFactory factory_;
  std::shared_ptr<Tree> tree_;
  std::unique_ptr<BT::PublisherZMQ> groot_monitor_;

  /// ROS Parameters
  std::string bt_file_path_;
  std::chrono::milliseconds loop_timeout_{};
  std::vector<std::string> plugins_;
  // Groot
  bool run_groot_monitoring_{};
  uint16_t publisher_port_{}, server_port_{}, max_msg_per_second_{};

  void configure_parameters();
  void load_tree();
  void run();
  void add_groot_monitoring();
  void load_plugins();

public:
  Px4MissionEngine();
};


#endif //PX4_MISSION_ENGINE_