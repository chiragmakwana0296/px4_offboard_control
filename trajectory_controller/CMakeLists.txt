cmake_minimum_required(VERSION 3.8)
project(trajectory_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(px4_control_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
)

add_executable(trajectory_control_server src/trajectory_control_server.cpp)
ament_target_dependencies(trajectory_control_server
  rclcpp
  rclcpp_action
  px4_ros_com
  px4_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  px4_control_interface
)
set_target_properties(trajectory_control_server PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)

# Install executable
install(TARGETS trajectory_control_server 
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(trajectory_planner src/trajectory_planner.cpp)
ament_target_dependencies(trajectory_planner
  rclcpp
  rclcpp_action
  px4_ros_com
  px4_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  px4_control_interface
)
set_target_properties(trajectory_planner PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)

# Install executable
install(TARGETS trajectory_planner 
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(commander_action src/commander_action.cpp)
ament_target_dependencies(commander_action
  rclcpp
  rclcpp_action
  px4_ros_com
  px4_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  px4_control_interface
)
set_target_properties(commander_action PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)

# Install executable
install(TARGETS commander_action 
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
