cmake_minimum_required(VERSION 3.8)
project(onero_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装config文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# # 安装worlds文件（如果有）
# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/worlds")
#   install(DIRECTORY worlds
#     DESTINATION share/${PROJECT_NAME}/
#   )
# endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

