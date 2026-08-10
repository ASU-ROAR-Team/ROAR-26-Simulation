cmake_minimum_required(VERSION 3.8)
project(dstar_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

add_library(dstar_planner SHARED src/dstar_planner.cpp)
ament_target_dependencies(dstar_planner
  rclcpp
  nav_msgs
  nav2_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  nav2_core
  pluginlib
)

add_executable(dstar_node src/dstar_node.cpp)
ament_target_dependencies(dstar_node
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS dstar_node dstar_planner
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core dstar_planner.xml)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/map_publisher.py
  ${PROJECT_NAME}/path_simulator.py
  scripts/heightmap_to_costmap.py
  scripts/erc_waypoint_follower.py
  test/maps/test_map_publisher.py
  test/harness/test_harness.py
  test/maps/dynamic_map_publisher.py
  test/harness/test_harness_dynamic.py
  test/harness/test_harness_fallback.py
  test/harness/test_harness_waypoints.py
  test/harness/test_harness_hybrid.py
  test/harness/test_harness_terrain.py
  test/harness/test_harness_erc.py
  test/maps/terrain_map_generator.py
  test/maps/erc_map_generator.py
  test/erc_simulation.py
  test/run_hybrid_tests.py
  test/run_terrain_tests.py
  test/run_fallback_test.py
  offline_tools/scripts/planning/offline_sequence_planner.py
  offline_tools/scripts/planning/dstar_tour_optimizer.py
  offline_tools/scripts/planning/optimize_waypoint_tour.py
  offline_tools/scripts/analysis/path_editor.py
  offline_tools/scripts/analysis/path_cost_evaluator.py
  offline_tools/scripts/analysis/compare_paths.py
  offline_tools/scripts/analysis/filter_path.py
  offline_tools/scripts/mapping/convert_heightmap_to_costmap.py
  offline_tools/scripts/mapping/generate_heightmap.py
  offline_tools/scripts/mapping/generate_path_attraction_costmap.py
  offline_tools/scripts/mapping/visualize_attraction_costmap.py
  offline_tools/scripts/mapping/map_calibration.py
  offline_tools/scripts/mapping/visualize_heightmap.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY offline_tools
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)

ament_package()