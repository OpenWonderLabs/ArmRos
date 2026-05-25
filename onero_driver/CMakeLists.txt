cmake_minimum_required(VERSION 3.8)
project(onero_driver)

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
find_package(onero_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# Architecture detection: prefer cache var, then env var, then host
if(NOT DEFINED ONERO_TARGET_ARCH OR ONERO_TARGET_ARCH STREQUAL "")
  if(DEFINED ENV{ONERO_TARGET_ARCH} AND NOT "$ENV{ONERO_TARGET_ARCH}" STREQUAL "")
    set(ONERO_TARGET_ARCH "$ENV{ONERO_TARGET_ARCH}")
  else()
    set(ONERO_TARGET_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
  endif()
endif()
string(TOLOWER "${ONERO_TARGET_ARCH}" _onero_arch_lc)
if(_onero_arch_lc STREQUAL "x86_64" OR _onero_arch_lc STREQUAL "amd64")
  set(ONERO_ARCH_TRIPLET "x86_64-linux-gnu")
  set(ONERO_PLATFORM_TAG "linux_x86_64_v1.0.0")
elseif(_onero_arch_lc STREQUAL "aarch64" OR _onero_arch_lc STREQUAL "arm64")
  set(ONERO_ARCH_TRIPLET "aarch64-linux-gnu")
  set(ONERO_PLATFORM_TAG "linux_aarch64_v1.0.0")
else()
  message(FATAL_ERROR "Unsupported ONERO_TARGET_ARCH: ${ONERO_TARGET_ARCH}")
endif()

include_directories(include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/third_party
)

# Per-arch third-party library dir; libonero_api itself is linked via explicit
# absolute path below to avoid ambiguity from the legacy top-level symlink.
set(THIRD_PARTY_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${ONERO_PLATFORM_TAG}/third_party/lib/${ONERO_ARCH_TRIPLET}")
link_directories(
  ${THIRD_PARTY_LIB_DIR}
)

# Add rpath-link for linker to find third-party libraries during linking
# This is needed because libonero_api.so depends on these libraries
if(EXISTS "${THIRD_PARTY_LIB_DIR}")
  message(STATUS "Found third-party libraries at: ${THIRD_PARTY_LIB_DIR}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath-link,${THIRD_PARTY_LIB_DIR}")
else()
  message(WARNING "Third-party libraries directory not found at: ${THIRD_PARTY_LIB_DIR}")
  message(WARNING "Please check that the onero_driver/lib runtime package contains bundled third-party libraries for this architecture")
endif()

# OneroDriver executable
add_executable(onero_driver_node src/onero_driver.cpp)
target_compile_definitions(onero_driver_node PRIVATE PINOCCHIO_WITH_HPP_FCL)
ament_target_dependencies(onero_driver_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  std_srvs
  onero_interfaces
)

# Link against the per-arch libonero_api.so explicitly. The top-level
# lib/libonero_api.so symlink is no longer load-bearing.
target_link_libraries(onero_driver_node
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/${ONERO_PLATFORM_TAG}/libonero_api.so
)

# Set RPATH so the executable can find libonero_api.so and third-party libraries at runtime
# $ORIGIN/.. means: relative to executable, go up one directory to lib/
# $ORIGIN/../third_party/lib/${ONERO_ARCH_TRIPLET}: third-party libraries location
# This works because:
# - executable: install/onero_driver/lib/onero_driver/onero_driver_node
# - library: install/onero_driver/lib/libonero_api.so
# - third-party: install/onero_driver/lib/third_party/lib/${ONERO_ARCH_TRIPLET}/*.so
# Note: libonero_api.so has its own RPATH to find third-party libs
set_target_properties(onero_driver_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/..;$ORIGIN/../third_party/lib/${ONERO_ARCH_TRIPLET}"
  BUILD_WITH_INSTALL_RPATH TRUE
)

install(TARGETS 
  onero_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  config 
  launch
  doc
  DESTINATION share/${PROJECT_NAME}
)

# Install SO library to install directory (always from per-arch dir).
install(FILES
  lib/${ONERO_PLATFORM_TAG}/libonero_api.so
  DESTINATION lib
)

# Install third-party libraries
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/${ONERO_PLATFORM_TAG}/third_party/lib/${ONERO_ARCH_TRIPLET}")
  install(DIRECTORY
    lib/${ONERO_PLATFORM_TAG}/third_party/lib/${ONERO_ARCH_TRIPLET}
    DESTINATION lib/third_party/lib
  )
endif()

# Install headers
install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_include_directories(include include/third_party)
ament_export_libraries(onero_api)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
