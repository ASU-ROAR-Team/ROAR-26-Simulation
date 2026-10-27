cmake_minimum_required(VERSION 3.8)
project(terrain_mapping)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python module (terrain_mapping/terrain_mapping_node.py)
ament_python_install_package(${PROJECT_NAME})

# Install the executable wrapper script
install(PROGRAMS
  scripts/terrain_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch + config
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install pre-baked heightmap data (.npz bakes + debug csv/png) so
# get_package_share_directory('terrain_mapping') resolves it after a
# colcon build on any machine/workspace layout, not just this one.
install(DIRECTORY
  working_heightmaps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
