#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from os import system
from tkinter import Tk, Event
import threading

class ControlApp(Tk, object):
    def __init__(self, ros_node) -> None:
        super(ControlApp, self).__init__()
        self.ros_node = ros_node
        self.config()
        self.initKeyboard()
        
    def initKeyboard(self) -> None:
        self.bind("<KeyPress>", self.keydown)
        self.bind("<KeyRelease>", self.keyup)
        
    def config(self) -> None:
        self.forw = 1.57
        self.stop = 0.0
        self.back = -1.57
        
    def keydown(self, event: Event) -> None:
        if event.keysym == "Up":
            self.ros_node.publish_velocities(self.forw, self.forw, self.forw, 
                                            self.forw, self.forw, self.forw)
        elif event.keysym == "Down":
            self.ros_node.publish_velocities(self.back, self.back, self.back,
                                            self.back, self.back, self.back)
        elif event.keysym == "Left":
            self.ros_node.publish_velocities(self.forw, self.back, self.forw,
                                            self.back, self.forw, self.back)
        elif event.keysym == "Right":
            self.ros_node.publish_velocities(self.back, self.forw, self.back,
                                            self.forw, self.back, self.forw)
    
    def keyup(self, event: Event) -> None:
        if event.keysym in ["Up", "Down", "Left", "Right"]:
            self.ros_node.publish_velocities(self.stop, self.stop, self.stop,
                                            self.stop, self.stop, self.stop)


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Create publishers
        self.velocitylfPublisher = self.create_publisher(
            Float64, '/wheel_lhs_front_velocity_controller/command', 10)
        self.velocityrfPublisher = self.create_publisher(
            Float64, '/wheel_rhs_front_velocity_controller/command', 10)
        self.velocitylrPublisher = self.create_publisher(
            Float64, '/wheel_lhs_rear_velocity_controller/command', 10)
        self.velocityrrPublisher = self.create_publisher(
            Float64, '/wheel_rhs_rear_velocity_controller/command', 10)
        self.velocitylmPublisher = self.create_publisher(
            Float64, '/wheel_lhs_mid_velocity_controller/command', 10)
        self.velocityrmPublisher = self.create_publisher(
            Float64, '/wheel_rhs_mid_velocity_controller/command', 10)
        
        self.get_logger().info('Teleop node started. Use arrow keys to control the rover.')
        
    def publish_velocities(self, lf, rf, lr, rr, lm, rm):
        """Publish velocities to all wheel controllers"""
        msg_lf = Float64()
        msg_lf.data = lf
        self.velocitylfPublisher.publish(msg_lf)
        
        msg_rf = Float64()
        msg_rf.data = rf
        self.velocityrfPublisher.publish(msg_rf)
        
        msg_lr = Float64()
        msg_lr.data = lr
        self.velocitylrPublisher.publish(msg_lr)
        
        msg_rr = Float64()
        msg_rr.data = rr
        self.velocityrrPublisher.publish(msg_rr)
        
        msg_lm = Float64()
        msg_lm.data = lm
        self.velocitylmPublisher.publish(msg_lm)
        
        msg_rm = Float64()
        msg_rm.data = rm
        self.velocityrmPublisher.publish(msg_rm)


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
