# Navigation & Mission Setup (`navMission_setup`)

This folder contains navigation mission configurations, waypoint lists, launch setups for Nav2 mapping, and mission setup packages for the ROAR simulation environment.

## Included Packages

- [**`rock_generator`**](./rock_generator/README.md): ROS 2 package for:
  - Generating obstacle dataset files (`obstacle_data.npy`) in the package `obs_data/` folder.
  - Fusing obstacle data into parameterized Gazebo `.world` files (e.g. `w_d0.011_c0.62.world`) and automatically creating their launcher files in the `worlds` package.
  - Launching simulation worlds and spawning/visualizing parameterized rocks in live Gazebo instances.

