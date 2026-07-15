# Mars Yard Physics Notes

## What is in the environment?

The Mars Yard is split into two meshes:

1. `mars_yard_only_textured.obj`  
   Visual only. This is what the user sees in Gazebo.

2. `mars_yard_collision_lowpoly.obj`  
   Collision only. This is invisible, but the rover wheels/body will collide with it and feel slopes, bumps, raised areas, and dips.

## Why not use the high-resolution mesh for collision?

The original ERC mesh is too heavy. Using it directly as collision in ROS 2 Humble / Ignition Fortress / DART can cause crashes or very slow simulation. The collision mesh is therefore low-poly.

## Current friction

The terrain uses:

- `mu = 1.05`
- `mu2 = 1.05`
- `slip1 = 0.015`
- `slip2 = 0.015`

This is a practical starting point for compact sand / rough soil. Increase `mu/mu2` if the rover slips too much on slopes. Decrease them if the rover grips unrealistically.

## Current physics

The world uses:

- `gravity = 9.81 m/s²`
- `max_step_size = 0.004`
- `real_time_update_rate = 250`

This balances stability and performance on Ubuntu 22.04 + ROS 2 Humble + Ignition Fortress.
