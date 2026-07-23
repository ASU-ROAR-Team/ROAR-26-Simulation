# Navigation & Mission Setup (`navMission_setup`)

This folder contains navigation mission configurations, waypoint lists, launch setups for Nav2 mapping, and mission setup packages for the ROAR simulation environment.

## Included Packages

- [**`rock_generator`**](./rock_generator/README.md): ROS 2 package for:
  - Generating obstacle dataset files (`obstacle_data.npy`) in `obs_data/`.
  - Fusing obstacle data into standalone Gazebo `.world` files in `Gen_worlds/`.
  - Launching simulation worlds (`marsyard.world`) and spawning/visualizing parameterized rocks in live Gazebo instances.
