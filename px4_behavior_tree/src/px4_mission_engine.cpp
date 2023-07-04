#include "px4_behavior_tree/px4_mission_engine.hpp"
#include <csignal>

Px4MissionEngine::Px4MissionEngine()
: Node("px4_mission_engine")
{
  configure_parameters();
  load_plugins();
  load_tree();
  if (run_groot_monitoring_) {
    add_groot_monitoring();
  }
  run();
}

void Px4MissionEngine::configure_parameters()
{
  bt_file_path_ = this->declare_parameter("bt_file_path", "/home/chirag/ws_sensor_combined/src/px4_offboard_control/px4_behavior_tree/trees/mission.xml");
  loop_timeout_ = std::chrono::milliseconds(this->declare_parameter("loop_timeout", 1500));
  plugins_ = this->declare_parameter("plugins", std::vector<std::string>());
  // Groot
  run_groot_monitoring_ = this->declare_parameter("run_groot_monitoring", true);
  publisher_port_ = this->declare_parameter("publisher_port", 1666);
  server_port_ = this->declare_parameter("server_port", 1667);
  max_msg_per_second_ = this->declare_parameter("max_msg_per_second", 25);
}

void Px4MissionEngine::load_tree()
{
  auto blackboard = Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", std::make_shared<rclcpp::Node>("bt_node"));
  RCLCPP_INFO_STREAM(this->get_logger(), "Loading tree from file: " + bt_file_path_);
  tree_ = std::make_shared<Tree>(factory_.createTreeFromFile(bt_file_path_, blackboard));
}

void Px4MissionEngine::run()
{
  rclcpp::WallRate loop_rate(loop_timeout_);
  RCLCPP_INFO_STREAM(
    this->get_logger(), "Running tree at frequency " <<
      1.0 / loop_timeout_.count() * 1e3 << "Hz");
  while (rclcpp::ok()) {
    tree_->tickOnce();
    loop_rate.sleep();
  }
}

void Px4MissionEngine::add_groot_monitoring()
{
  RCLCPP_INFO_STREAM(
    this->get_logger(), "Groot monitoring enabled with server port [" <<
      server_port_ << "] and publisher port [" << publisher_port_ << "]");
  groot_monitor_ = std::make_unique<BT::PublisherZMQ>(
    *tree_, max_msg_per_second_, publisher_port_, server_port_);
}

void Px4MissionEngine::load_plugins()
{
  for (const auto & p : plugins_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Loading plugin: " + SharedLibrary::getOSName(p));
    factory_.registerFromPlugin(SharedLibrary::getOSName(p));
  }
}

void sigint_handler(__attribute__((unused)) int signal_num) { // Silences compiler warnings
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  signal(SIGINT, sigint_handler);
  rclcpp::spin(std::make_shared<Px4MissionEngine>());
  rclcpp::shutdown();
  return 0;
}