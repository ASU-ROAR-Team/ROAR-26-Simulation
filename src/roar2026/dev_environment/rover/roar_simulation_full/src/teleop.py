#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from os import system
from tkinter import Tk, Event, Label
import threading

class ControlApp(Tk, object):
    def __init__(self, ros_node) -> None:
        super(ControlApp, self).__init__()
        self.ros_node = ros_node
        self.config_vars()
        self._held_keys = set()
        self._release_timers = {}
        self.initKeyboard()
        self.title("Rover Teleop")
        self.geometry("300x100")
        
        lbl = Label(self, text="Rover Teleop Active\n\nClick here to focus window.\nUse Arrow Keys to drive.", font=("Arial", 12))
        lbl.pack(expand=True)
        
    def initKeyboard(self) -> None:
        self.bind("<KeyPress>", self.keydown)
        self.bind("<KeyRelease>", self.keyup)
        self.bind("<FocusOut>", self._on_focus_out)
        
    def config_vars(self) -> None:
        self.linear_speed = 1.0   # m/s
        self.angular_speed = 1.0  # rad/s

    def _apply_held_keys(self) -> None:
        lin = 0.0
        ang = 0.0
        if "Up" in self._held_keys:
            lin += self.linear_speed
        if "Down" in self._held_keys:
            lin -= self.linear_speed
        if "Left" in self._held_keys:
            ang += self.angular_speed
        if "Right" in self._held_keys:
            ang -= self.angular_speed
        self.ros_node.update_velocities(lin, ang)

    def _on_focus_out(self, _event: Event) -> None:
        self._held_keys.clear()
        for tid in self._release_timers.values():
            self.after_cancel(tid)
        self._release_timers.clear()
        self._apply_held_keys()
        
    def keydown(self, event: Event) -> None:
        key = event.keysym
        if key not in ("Up", "Down", "Left", "Right"):
            return
        if key in self._release_timers:
            self.after_cancel(self._release_timers.pop(key))
        if key not in self._held_keys:
            self._held_keys.add(key)
            self._apply_held_keys()
    
    def keyup(self, event: Event) -> None:
        key = event.keysym
        if key not in ("Up", "Down", "Left", "Right"):
            return
        if key in self._release_timers:
            self.after_cancel(self._release_timers[key])
        # Debounce synthetic X11/XWayland key-repeat release events (~0-5 ms)
        self._release_timers[key] = self.after(50, lambda k=key: self._confirm_release(k))

    def _confirm_release(self, key: str) -> None:
        self._release_timers.pop(key, None)
        if key in self._held_keys:
            self._held_keys.discard(key)
            self._apply_held_keys()

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # ROS2 diff_drive_controller expects Twist on this topic by default
        self.cmd_vel_pub_1 = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.cmd_vel_pub_2 = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel', 10)
        self.cmd_vel_pub_3 = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        
        # Publish at 10Hz to keep the controller alive
        self.timer = self.create_timer(0.1, self.publish_velocities)
        
        self.get_logger().info('Teleop node started. Select the GUI window and use arrow keys!')
        
    def update_velocities(self, linear_x, angular_z):
        self.current_linear_x = float(linear_x)
        self.current_angular_z = float(angular_z)
        self.publish_velocities() # Publish immediately for low latency

    def publish_velocities(self):
        msg = Twist()
        msg.linear.x = self.current_linear_x
        msg.angular.z = self.current_angular_z
        self.cmd_vel_pub_1.publish(msg)
        self.cmd_vel_pub_2.publish(msg)
        self.cmd_vel_pub_3.publish(msg)

def ros_spin_thread(node):
    """Spin ROS2 node in a separate thread"""
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        system('xset r off')
        
        # Create ROS2 node
        teleop_node = TeleopNode()
        
        # Start ROS2 spinning in a separate thread
        spin_thread = threading.Thread(target=ros_spin_thread, args=(teleop_node,), daemon=True)
        spin_thread.start()
        
        # Create and run Tkinter GUI in main thread
        control = ControlApp(teleop_node)
        control.mainloop()
        
        system('xset r on')
        
    except KeyboardInterrupt:
        system('xset r on')
        pass
    finally:
        teleop_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
