# 🔌 Referee Engine Integration Guide

This document is designed for developers who are ready to integrate their own Navigation Stack (e.g., Nav2, custom controllers, planners) with the Automated Referee Engine.

---

## 1. How the Referee Engine Evaluates Your Rover

The Referee Engine uses a **Velocity-Triggered Parking Method**. It does not instantly lock your score the moment you touch a waypoint. Instead, it allows your nav stack to drive as close to the center as possible.

### The Acceptance Logic:
1. Your rover navigates toward the target waypoint.
2. The rover crosses into the `acceptance_radius` (default: `0.2m`). The referee begins tracking the distance internally but takes no action yet.
3. Your nav stack parks the rover. When the rover's speed drops below `0.05 m/s` while inside the radius, the referee triggers the "Waypoint Reached" state!
4. The referee records your `Dmin` (the absolute closest your rover ever got to the exact center point of the waypoint).
5. The referee pauses the mission clock (so you don't lose time points) and waits for a human judge to verify the parking.

---

## 2. Topic Interfaces

Your system must communicate with the Referee Engine over several ROS 2 topics. 

### What Your System MUST Publish:
| Topic | Message Type | Description |
|---|---|---|
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | The referee uses this to calculate the rover's current $(X, Y)$ position and its velocity. *Note: If `run_mock:=false`, the launch file automatically listens to this topic.* |
| `/clock` | `rosgraph_msgs/Clock` | Simulation time. The referee uses this to accurately calculate your driving time and penalize timeouts. |
| `/chassis/contact_sensors` | `gazebo_msgs/ContactsState` | Publishes collision events if your rover hits a rock. The referee uses this to immediately terminate the mission with `0.0` points if a fatal collision occurs. |

### What the Referee Publishes (For your system to listen to):
| Topic | Message Type | Description |
|---|---|---|
| `/referee/status` | `std_msgs/String` | Real-time state strings: `RUNNING`, `WAITING_FOR_COMMAND`, `COMPLETED`, `TIMEOUT`, `FAILED`. |
| `/referee/score` | `std_msgs/String` | A massive JSON payload containing the complete mathematical breakdown of every waypoint, broadcasted only when the mission concludes. |

---

## 3. Commanding the Referee (Fully Autonomous)

Because the referee simulates a real-life human judge, it will freeze the mission when you park at a waypoint. Your nav stack cannot proceed to the next waypoint until the referee unlocks it. 

While you can manually type this in a terminal (`ros2 topic pub --once /referee/command std_msgs/String "{data: 'NEXT'}"`), your **Real Navigation System** should handle this entirely autonomously!

### How to automate it in your Nav Stack:
1. **Listen to the Referee**: Have your main autonomy node subscribe to the `/referee/status` topic.
2. **Detect the Pause**: When your nav stack finishes parking, wait for the referee to broadcast `"WAITING_FOR_COMMAND"` on the status topic. This confirms the referee recognized your park and locked your score!
3. **Wait & Publish**: Have your autonomy node wait for a few seconds (e.g., 5 seconds to simulate a judge's measurement), and then programmatically publish `"NEXT"` to `/referee/command`.

Here is a quick Python snippet you can drop straight into your navigation autonomy node to handle this:

```python
from std_msgs.msg import String

# Inside your autonomy node's __init__:
self.command_pub = self.create_publisher(String, '/referee/command', 10)

def unlock_referee(self):
    """Call this function when you are ready to drive to the next waypoint!"""
    msg = String()
    msg.data = "NEXT"
    self.command_pub.publish(msg)
    self.get_logger().info("Sent NEXT command to referee. Unlocking next waypoint...")
```

---

## 4. Customizing the Configuration

You can easily adjust the strictness of the referee by modifying the parameters in `config/scoring_config.yaml`:
* `acceptance_radius`: How close you must get to the waypoint before you are allowed to stop (default `0.2m`).
* `max_points_per_waypoint`: Maximum score awarded for a dead-center park (default `50.0`).
* `timeout_buffer_multiplier`: Multiplier applied to the "Perfect Time" to calculate your absolute maximum time limit (default `1.5x`).
