# Simulation Collision & Contact Sensors Architecture

This document serves as the technical guide for the collision detection system implemented in the ROAR Simulation Dev Environment. It explains how Gazebo contacts are bridged to ROS 2 and intelligently filtered using a programmable blacklist.

## 1. Gazebo Sensor Configuration (`.xacro` files)
Contact sensors have been attached to the rover's physical collision meshes (the base link and all 6 wheels). This was injected via macros in the `rover_gazebo.xacro` files.

**How it works in Ignition Gazebo:**
Gazebo's contact sensor generates raw `ignition.msgs.Contacts` messages. Because Ignition dynamically creates a unique topic for every single sensor based on the entity tree, the raw topics output in a very long format, like so:
`/world/rover_world/model/roar_rover/link/wheel_lhs_front/sensor/wheel_lhs_front_contact/contact`

## 2. ROS 2 Bridge Integration (`basic_rover.launch.py`)
To make these 7 individual Gazebo topics usable in our ROS 2 stack, they are passed through the `ros_gz_bridge`.

Inside all `basic_rover.launch.py` variants, the bridge is configured to:
1. Subscribe to all 7 raw Gazebo contact topics.
2. Translate the `ignition.msgs.Contacts` messages into `ros_gz_interfaces/msg/Contacts`.
3. **Remap** all 7 topics into a single, unified ROS 2 topic: **`/rover_contact`**.

This means your navigation algorithms only ever need to listen to `/rover_contact` to know if *any* part of the rover has experienced a physical impact.

## 3. The Centralized Collision Alarm Node
Because Gazebo's physics engine is extremely literal, the rover's wheels touching the ground is registered as a continuous, permanent collision. If we directly listened to the raw contact sensors, the system would spam collision warnings 50 times a second just from sitting on the Marsyard.

To solve this, we created the **`collision_alarm_node.py`** (located in the `src/` directory of the active rover package).
- **Input:** Subscribes to `/rover_contact`.
- **Output:** Publishes a simple `std_msgs/Bool` on **`/collision_alarm`**. It outputs `True` when crashing, and `False` when clear.

### The Blacklist Filtering Approach
The alarm node uses a programmable blacklist to intelligently filter out harmless contacts and ground terrain. 

When a collision occurs, the message contains the names of the two objects that touched (e.g., `wheel_rhs_front_collision` and `mars_yard_collision`). The Python node evaluates these names against a safe-list:

```python
# Inside collision_alarm_node.py
c1 = contact.collision1.name.lower()
c2 = contact.collision2.name.lower()

# Blacklist: Ignore any collision involving these safe keywords
if 'mars_yard' not in c1 and 'ground' not in c1 and 'terrain' not in c1:
    # If the safe keywords are NOT present in the collision name, it's a REAL collision!
    trigger_alarm()
```

### Extending the Filter (For Other Teams)
If your team adds new non-collidable objects to the simulation (for example: a holographic marker, a safe-zone bounding box, or a soft obstacle) and you notice it is triggering the `/collision_alarm` when the rover drives through it:

1. Open `src/collision_alarm_node.py` in your active rover package.
2. Locate the `if` statement checking `c1` and `c2`.
3. Add your new object's name to the blacklist: `and 'hologram' not in c1`.
4. Rebuild the workspace: `colcon build`.

### Using a Whitelist Approach Instead
Alternatively, if you want to invert this behavior so the alarm *only* triggers for specific hazards and ignores absolutely everything else, you can modify the node to check a list of hazardous objects instead:
```python
hazardous_objects = ['rock', 'wall', 'crater_edge']

if any(danger in c1 for danger in hazardous_objects) or any(danger in c2 for danger in hazardous_objects):
    trigger_alarm()
```
