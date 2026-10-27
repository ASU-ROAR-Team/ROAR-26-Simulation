# Creating New Simulation Worlds for the ROAR NavStack

If you are generating or creating new `.world` (SDF) files for the rover to navigate in, they **must** contain specific configurations and plugins to be fully compatible with the 2026 Navigation Stack. 

Failure to include these will result in missing sensor data (IMU, collisions, cameras) or localization failures (teleporting, drift) because the ROS 2 bridges will not find the correct Gazebo topics.

## 1. World Name (Critical for Localization)
The world tag inside the SDF **must** be named exactly `"rover_world"`. 
This is because our localization stack (`ground_truth_bridge`) explicitly listens to `/world/rover_world/pose/info` to generate perfect ZED pose tracking.

```xml
<!-- CORRECT -->
<world name="rover_world">

<!-- INCORRECT - Will break navigation tracking! -->
<world name="my_custom_world"> 
```

## 2. Essential Gazebo Systems Plugins
Every new world must include the following `ignition-gazebo` plugins inside the `<world>` tag for the rover's sensors and physics to function properly:

```xml
    <!-- Basic Physics and Interaction -->
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics" />
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands" />
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster" />
    
    <!-- Collisions and Contacts (Required for the Collision Alarm Node) -->
    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact" />
    
    <!-- Ground Truth Pose Publisher (Required for IESKF ZED Simulation) -->
    <plugin filename="ignition-gazebo-pose-publisher-system" name="ignition::gazebo::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_nested_model_pose>false</publish_nested_model_pose>
      <use_pose_vector_msg>true</use_pose_vector_msg>
      <update_frequency>50</update_frequency>
    </plugin>
    
    <!-- Sensors (Required for Cameras, LiDAR, IMU) -->
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-magnetometer-system" name="ignition::gazebo::systems::Magnetometer"/>
```

## 3. Include the Terrain/Environment
Don't forget to include the static mesh or heightmap for the terrain. Example:
```xml
    <include>
      <uri>model://mars_yard</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
```

If you follow these 3 rules, your new world will flawlessly integrate with the batch GUI, the simulation launchers, and the Nav2 path planners.
