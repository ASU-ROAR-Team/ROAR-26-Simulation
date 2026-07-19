cmake_minimum_required(VERSION 3.8)
project(roar_simulation_full)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(robot_state_publisher REQUIRED)
#find_package(joint_state_publisher_gui REQUIRED)

# Install Python scripts
install(PROGRAMS
  src/teleop.py
  src/ground_truth_tf_broadcaster.py
  src/zed_degradation_node.py
  src/clean_joint_state_publisher.py
  src/encoder_sim_node.py
  src/clean_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  configs
  launch
  meshes
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install model.config
install(FILES
  model.config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
