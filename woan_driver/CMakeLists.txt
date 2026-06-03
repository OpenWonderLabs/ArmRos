cmake_minimum_required(VERSION 3.8)
project(woan_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Suppress noisy warnings for cleaner build output
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-sign-compare -Wno-reorder -Wno-class-memaccess)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(woan_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/third_party
)

# Add explicit SO library location for local woan_api
# Also add third-party libraries directory for linking
set(THIRD_PARTY_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/linux_x86_64_v1.0.0/third_party/lib/x86_64-linux-gnu")
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
  ${THIRD_PARTY_LIB_DIR}
)

# Add rpath-link for linker to find third-party libraries during linking
# This is needed because libwoan_api.so depends on these libraries
if(EXISTS "${THIRD_PARTY_LIB_DIR}")
  message(STATUS "Found third-party libraries at: ${THIRD_PARTY_LIB_DIR}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath-link,${THIRD_PARTY_LIB_DIR}")
else()
  message(WARNING "Third-party libraries directory not found at: ${THIRD_PARTY_LIB_DIR}")
  message(WARNING "Please run lib_install.sh in woan_api_src to install the bundled libraries")
endif()

# WoanDriver executable
add_executable(woan_driver_node src/woan_driver.cpp)
target_compile_definitions(woan_driver_node PRIVATE PINOCCHIO_WITH_HPP_FCL)
ament_target_dependencies(woan_driver_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  std_srvs
  woan_interfaces
)

# Link against woan_api SO library (local path)
target_link_libraries(woan_driver_node
  woan_api
)

# Set RPATH so the executable can find libwoan_api.so and third-party libraries at runtime
# $ORIGIN/.. means: relative to executable, go up one directory to lib/
# $ORIGIN/../third_party/lib/x86_64-linux-gnu: third-party libraries location
# This works because:
# - executable: install/woan_driver/lib/woan_driver/woan_driver_node
# - library: install/woan_driver/lib/libwoan_api.so
# - third-party: install/woan_driver/lib/third_party/lib/x86_64-linux-gnu/*.so
# Note: libwoan_api.so has its own RPATH to find third-party libs
set_target_properties(woan_driver_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/..;$ORIGIN/../third_party/lib/x86_64-linux-gnu"
  BUILD_WITH_INSTALL_RPATH TRUE
)

install(TARGETS 
  woan_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  config 
  launch
  doc
  DESTINATION share/${PROJECT_NAME}
)

# Install SO library to install directory
# Install the actual library file (follow symlink if exists)
# Check if libwoan_api.so is a symlink and install the real file
if(IS_SYMLINK "${CMAKE_CURRENT_SOURCE_DIR}/lib/libwoan_api.so")
  get_filename_component(REAL_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/libwoan_api.so" REALPATH)
  install(FILES 
    ${REAL_LIB}
    DESTINATION lib
    RENAME libwoan_api.so
  )
else()
  install(FILES 
    lib/libwoan_api.so
    DESTINATION lib
  )
endif()

# Install third-party libraries
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/linux_x86_64_v1.0.0/third_party/lib/x86_64-linux-gnu")
  install(DIRECTORY
    lib/linux_x86_64_v1.0.0/third_party/lib/x86_64-linux-gnu
    DESTINATION lib/third_party/lib
  )
endif()

# Install headers
install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_include_directories(include include/third_party)
ament_export_libraries(woan_api)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
