cmake_minimum_required(VERSION 3.8)
project(onero_examples)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Suppress noisy warnings for cleaner build output
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-sign-compare)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(onero_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_package(onero_driver REQUIRED)
find_package(tf2 REQUIRED)
find_package(trajectory_msgs REQUIRED)#jingyi

# Move API All示例
add_executable(move_all src/move_api_all.cpp)
ament_target_dependencies(move_all
  rclcpp
  std_msgs
  geometry_msgs
  onero_interfaces
  onero_driver
)

# Move API All示例
install(PROGRAMS
  src/move_api_all.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_api_all_py
)

# MoveJ示例
add_executable(movej_demo src/movej_demo.cpp)
ament_target_dependencies(movej_demo
  rclcpp
  std_msgs
  onero_interfaces
  onero_driver
)

# MoveL示例
add_executable(movel_demo src/movel_demo.cpp)
ament_target_dependencies(movel_demo
  rclcpp
  std_msgs
  geometry_msgs
  onero_interfaces
  onero_driver
)

# MoveP示例
add_executable(movep_demo src/movep_demo.cpp)
ament_target_dependencies(movep_demo
  rclcpp
  geometry_msgs
  onero_interfaces
  onero_driver
)

# 状态监控示例
add_executable(state_monitor src/state_monitor.cpp)
ament_target_dependencies(state_monitor
  rclcpp
  std_msgs
  onero_interfaces
  onero_driver
  tf2
)

# 轨迹连接示例
add_executable(trajectory_connect_demo src/trajectory_connect_demo.cpp)
ament_target_dependencies(trajectory_connect_demo
  rclcpp
  std_msgs
  onero_interfaces
  onero_driver
)

# 拖动示教节点
add_executable(drag_teaching_node src/drag_teaching_node.cpp)
ament_target_dependencies(drag_teaching_node
  rclcpp
  std_msgs
  sensor_msgs
  onero_interfaces
  onero_driver
)

target_include_directories(drag_teaching_node PRIVATE ${EIGEN3_INCLUDE_DIRS})

# 双臂综合示例（仅在双臂模式下运行）
add_executable(dual_arm_demo src/dual_arm_demo.cpp)
ament_target_dependencies(dual_arm_demo
  rclcpp
  std_msgs
  onero_interfaces
  onero_driver
)

# 双臂拖动示教节点（仅在双臂模式下运行）
add_executable(dual_arm_drag_teaching_node src/dual_arm_drag_teaching_node.cpp)
ament_target_dependencies(dual_arm_drag_teaching_node
  rclcpp
  std_msgs
  sensor_msgs
  onero_interfaces
  onero_driver
)
target_include_directories(dual_arm_drag_teaching_node PRIVATE ${EIGEN3_INCLUDE_DIRS})

# 单臂带夹爪示例（仅在单臂 + 启用夹爪时运行）
add_executable(single_arm_gripper_demo src/single_arm_gripper_demo.cpp)
ament_target_dependencies(single_arm_gripper_demo
  rclcpp
  onero_interfaces
  onero_driver
)

# 双臂带夹爪示例（仅在双臂 + 启用两侧夹爪时运行）
add_executable(dual_arm_gripper_demo src/dual_arm_gripper_demo.cpp)
ament_target_dependencies(dual_arm_gripper_demo
  rclcpp
  onero_interfaces
  onero_driver
)

# 安装Python脚本
install(PROGRAMS
  src/trajectory_connect_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME trajectory_connect_demo_py
)

install(TARGETS
  move_all
  movej_demo
  movel_demo
  movep_demo
  state_monitor
  trajectory_connect_demo
  drag_teaching_node
  dual_arm_demo
  dual_arm_drag_teaching_node
  single_arm_gripper_demo
  dual_arm_gripper_demo
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY 
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
