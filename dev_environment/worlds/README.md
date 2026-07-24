# Adding New Worlds to the Rover Simulation

If you want to add a new `.world` file to the simulation (e.g., `world4.world`), you must make two critical modifications inside the XML structure of the file before it will work correctly with the rover's sensor pipeline.

These changes are required because the sun glare degradation node and other ground-truth dependent systems rely on a very specific Gazebo topic (`/world/rover_world/pose/info`) to calculate the exact positions of objects in the world.

### 1. Rename the World to `rover_world`
The ROS 2 bridges and python scripts are hardcoded to look for a world named `rover_world`. If your world is named `marsyard` or `default`, the bridge will fail to connect.

Open your `.world` file and change the opening `<world>` tag:

```xml
<!-- CHANGE THIS -->
<world name="default"> 

<!-- TO THIS -->
<world name="rover_world">
```

### 2. Add the PosePublisher Plugin
To calculate the dynamic sun glare effect on the ZED2 camera, the simulation needs to know the exact 3D coordinates of the Sun marker and the Rover in real-time. 

You must inject the `PosePublisher` plugin **inside** the `<world ...>` tags. Place this snippet right after the `<physics>` or `<scene>` tags near the top of the file:

```xml
    <!-- Required for TF Ground Truth (Sun Glare & Degradation Nodes) -->
    <plugin filename="ignition-gazebo-pose-publisher-system" name="ignition::gazebo::systems::PosePublisher">
      <publish_link_pose>true</publish_link_pose>
      <publish_visual_pose>false</publish_visual_pose>
      <publish_collision_pose>false</publish_collision_pose>
      <publish_sensor_pose>true</publish_sensor_pose>
      <publish_nested_model_pose>true</publish_nested_model_pose>
      <use_pose_vector_msg>true</use_pose_vector_msg>
      <static_publisher>false</static_publisher>
      <static_update_frequency>1</static_update_frequency>
    </plugin>
```

### 3. Verify
After adding the new world to your launch script (e.g., `launch_test.sh noise world4 rviz`), run the simulation. 
If you did everything correctly, you should be able to run:
```bash
ros2 topic echo /world/rover_world/pose/info
```
and see a constant stream of poses for `roar_rover` and `sun_marker`. If you do not see this stream, the camera sun glare will silently disable itself!
