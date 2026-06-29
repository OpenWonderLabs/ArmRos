cmake_minimum_required(VERSION 3.8)
project(onero_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(onero_interfaces REQUIRED)

# onero_control_node executable
add_executable(onero_control_node src/onero_control.cpp)
add_executable(onero_control_dual_node src/onero_control_dual.cpp)
ament_target_dependencies(onero_control_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  moveit_ros_planning_interface
  moveit_msgs
  onero_interfaces
)
ament_target_dependencies(onero_control_dual_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  moveit_ros_planning_interface
  moveit_msgs
  onero_interfaces
)

install(TARGETS 
  onero_control_node
  onero_control_dual_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
