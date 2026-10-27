cmake_minimum_required(VERSION 3.8)
project(roar_localization)

# ── C++ standard ──────────────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(roar_msgs REQUIRED)

# ── Includes ──────────────────────────────────────────────────────────────────
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# ─────────────────────────────────────────────────────────────────────────────
# Core filter library (pure C++ — no ROS), unit-testable
# ─────────────────────────────────────────────────────────────────────────────
add_library(roar_filter
  src/ieskf.cpp
  src/wio.cpp
  src/wvo.cpp
  src/aruco_map.cpp
)
target_include_directories(roar_filter PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(roar_filter
  ${YAML_CPP_LIBRARIES}
)

# ─────────────────────────────────────────────────────────────────────────────
# Main IESKF node (ROS2 executable)
# ─────────────────────────────────────────────────────────────────────────────
add_executable(ieskf_node
  src/ieskf_node.cpp
)
target_link_libraries(ieskf_node
  roar_filter
)
ament_target_dependencies(ieskf_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
  tf2
  roar_msgs
)

# ─────────────────────────────────────────────────────────────────────────────
# ArUco simulator node (simulation-only)
# ─────────────────────────────────────────────────────────────────────────────
add_executable(aruco_simulator_node
  src/aruco_simulator_node.cpp
)
target_link_libraries(aruco_simulator_node
  roar_filter
)
ament_target_dependencies(aruco_simulator_node
  rclcpp
  geometry_msgs
  std_msgs
  roar_msgs
)

# ─────────────────────────────────────────────────────────────────────────────
# Simulation helper nodes (sim-only: fake ZED pose + fake IMU from ground truth)
# ─────────────────────────────────────────────────────────────────────────────
add_executable(ground_truth_bridge
  src/ground_truth_bridge.cpp
)
ament_target_dependencies(ground_truth_bridge
  rclcpp
  geometry_msgs
  tf2
  tf2_msgs
)

add_executable(synthetic_imu_node
  src/synthetic_imu_node.cpp
)
target_include_directories(synthetic_imu_node PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(synthetic_imu_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

# ─────────────────────────────────────────────────────────────────────────────
# Unit tests
# ─────────────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  add_executable(test_math_utils  test/test_math_utils.cpp)
  target_include_directories(test_math_utils  PRIVATE include ${EIGEN3_INCLUDE_DIR})

  add_executable(test_ieskf       test/test_ieskf.cpp)
  target_link_libraries(test_ieskf  roar_filter)

  add_executable(test_wio         test/test_wio.cpp)
  target_link_libraries(test_wio    roar_filter)

  add_executable(test_wvo         test/test_wvo.cpp)
  target_link_libraries(test_wvo    roar_filter)

  add_executable(test_zupt        test/test_zupt.cpp)
  target_link_libraries(test_zupt   roar_filter)

  add_executable(test_aruco       test/test_aruco.cpp)
  target_link_libraries(test_aruco  roar_filter)
endif()

# ─────────────────────────────────────────────────────────────────────────────
# Install
# ─────────────────────────────────────────────────────────────────────────────
install(TARGETS
  ieskf_node
  aruco_simulator_node
  ground_truth_bridge
  synthetic_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/zed_pose_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
