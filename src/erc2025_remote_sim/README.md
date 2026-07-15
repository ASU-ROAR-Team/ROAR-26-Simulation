# Sapienza Technology Team Gazebo environment for ERC2025 Remote formula

## Documentation

[The documentation is available here](https://docs.google.com/document/d/13j-jUbbqSdOE2rpgMe3CeMcOyCfnu6AFhSe7TqYO01E/edit?usp=sharing)

With description of Gazebo version used and detailed workflow on how the 3D model of the marsyard was processed for the simulation.

Feel free to report any issue or suggest improvements!


## Install

Follow the instructions to install husarion simulation environment:
https://github.com/husarion/husarion_ugv_ros/tree/ros2-devel 

We tested with the ros2-devel branch of husarion repository so other branches are not guaranteed to work.

Clone and build this package:

```bash
    cd <your_workspace>/src
    git clone https://github.com/Sapienza-Technology-Team-Remote/erc2025_remote_sim.git
    cd ..
    colcon build
```

## Running

Run the Gazebo simulation using:

    ros2 launch erc2025_remote_sim startsimulation.launch.py 


## Tuning

Select how many cameras you want to use by modifying the config/components.yaml file. You can comment out cameras or add them.


By modifying the URDF file of the zed from the Husarion repository it is possible to:
- improve a lot performances by removing collisions for the ZED camera
- Fix the pointcloud being sideways
- optionally change the camera resolution
  
The file to modify is located in the Husarion packages at ``ros_components_description/urdf/stereolabs_zed.urdf.xacro``

You can replace the original file with the file located in this repository at ``config/zed_modified.urdf.xacro``

### Changelog of what has been modified

#### Commented out collision tag of the camera model:

```xml
      <!-- 
      <collision>
        <geometry>
          <mesh filename="package://ros_components_description/meshes/${model}.stl" />
        </geometry>
      </collision> 
      -->
```

#### Remove rotation component from ZED optical center frame:

```xml
    <link name="${device_namespace}_center_optical_frame" />
    <joint name="${device_namespace}_center_joint" type="fixed">
      <parent link="${device_namespace}_center" />
      <child link="${device_namespace}_center_optical_frame" />
      <!-- <origin xyz="0.0 0.0 0.0" rpy="${-pi/2.0} 0 ${-pi/2.0}" /> -->
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
    </joint>
```

#### Optional: change FPS and resolution
```xml
      <sensor type="camera" name="${ns}${device_namespace}_stereolabs_zed_color">
        <always_on>true</always_on>
        <!-- CHANGE FPS HERE -->
        <update_rate>15.0</update_rate>

        <topic>${ns}${device_namespace}/zed_node/rgb/image_rect_color</topic>
        <visualize>false</visualize>

        <gz_frame_id>${ns}${device_namespace}_center_optical_frame</gz_frame_id>
        <camera>
          <horizontal_fov>${110.0/180.0*pi}</horizontal_fov>
          <image>
            <!-- CHANGE RESOLUTION HERE -->
            <width>960</width>
            <height>640</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.02</near>
            <far>300.0</far>
          </clip>
        </camera>
      </sensor>
```
**Remember to apply the same changes also to the depth camera component.**

## TODO
- [ ] Integrate ZED changes without modifications of official Husarion repo
- [ ] ZED pointcloud is not rgb
- [ ] Add landmarks
- [ ] Add random objects
- [ ] Add configuration for Challenge 2?

