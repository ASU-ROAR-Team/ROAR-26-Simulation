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
        self.initKeyboard()
        self.title("Rover Teleop")
        self.geometry("300x100")
        
        lbl = Label(self, text="Rover Teleop Active\n\nClick here to focus window.\nUse Arrow Keys to drive.", font=("Arial", 12))
        lbl.pack(expand=True)
        
    def initKeyboard(self) -> None:
        self.bind("<KeyPress>", self.keydown)
        self.bind("<KeyRelease>", self.keyup)
        
    def config_vars(self) -> None:
        self.linear_speed = 1.0   # m/s
        self.angular_speed = 1.0  # rad/s
        
    def keydown(self, event: Event) -> None:
        if event.keysym == "Up":
            self.ros_node.update_velocities(self.linear_speed, 0.0)
        elif event.keysym == "Down":
            self.ros_node.update_velocities(-self.linear_speed, 0.0)
        elif event.keysym == "Left":
            self.ros_node.update_velocities(0.0, self.angular_speed)
        elif event.keysym == "Right":
            self.ros_node.update_velocities(0.0, -self.angular_speed)
    
    def keyup(self, event: Event) -> None:
        if event.keysym in ["Up", "Down", "Left", "Right"]:
            self.ros_node.update_velocities(0.0, 0.0)

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
        rclpy.shutdown()

if __name__ == "__main__":
    main()
