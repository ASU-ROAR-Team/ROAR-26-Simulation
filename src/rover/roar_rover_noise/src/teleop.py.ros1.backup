#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8MultiArray, Float64
from os import system
from tkinter import Tk, Event

class ControlApp(Tk, object):

    def __init__(self) -> None:
        super(ControlApp, self).__init__()
        self.initNode()
        self.config()
        self.initKeyboard()
        self.mainloop()

    def initNode(self) -> None:
        rospy.init_node("teleop_node")
        self.velocitylfPublisher = rospy.Publisher('/wheel_lhs_front_velocity_controller/command', Float64, queue_size=10)
        self.velocityrfPublisher = rospy.Publisher('/wheel_rhs_front_velocity_controller/command', Float64, queue_size=10)
        self.velocitylrPublisher = rospy.Publisher('/wheel_lhs_rear_velocity_controller/command', Float64, queue_size=10)
        self.velocityrrPublisher = rospy.Publisher('/wheel_rhs_rear_velocity_controller/command', Float64, queue_size=10)
        self.velocitylmPublisher = rospy.Publisher('/wheel_lhs_mid_velocity_controller/command', Float64, queue_size=10)
        self.velocityrmPublisher = rospy.Publisher('/wheel_rhs_mid_velocity_controller/command', Float64, queue_size=10)
        
    def initKeyboard(self) -> None:
        self.bind("<KeyPress>", self.keydown)
        self.bind("<KeyRelease>", self.keyup)

    def config(self) -> None:
        self.forw = 1.57
        self.stop = 0
        self.back = -1.57
        self.speedsMsg = Int8MultiArray()

    def keydown(self, event: Event) -> None:
        if event.keysym == "Up":
            self.velocitylfPublisher.publish(self.forw)
            self.velocityrfPublisher.publish(self.forw)
            self.velocitylrPublisher.publish(self.forw)
            self.velocityrrPublisher.publish(self.forw)
            self.velocitylmPublisher.publish(self.forw)
            self.velocityrmPublisher.publish(self.forw)
        elif event.keysym == "Down":
            self.velocitylfPublisher.publish(self.back)
            self.velocityrfPublisher.publish(self.back)
            self.velocitylrPublisher.publish(self.back)
            self.velocityrrPublisher.publish(self.back)
            self.velocitylmPublisher.publish(self.back)
            self.velocityrmPublisher.publish(self.back)
        elif event.keysym == "Left":
            self.velocitylfPublisher.publish(self.forw)
            self.velocityrfPublisher.publish(self.back)
            self.velocitylrPublisher.publish(self.forw)
            self.velocityrrPublisher.publish(self.back)
            self.velocitylmPublisher.publish(self.forw)
            self.velocityrmPublisher.publish(self.back)
        elif event.keysym == "Right":
            self.velocitylfPublisher.publish(self.back)
            self.velocityrfPublisher.publish(self.forw)
            self.velocitylrPublisher.publish(self.back)
            self.velocityrrPublisher.publish(self.forw)
            self.velocitylmPublisher.publish(self.back)
            self.velocityrmPublisher.publish(self.forw)

    def keyup(self, event: Event) -> None:
        if event.keysym in ["Up", "Down", "Left", "Right"]:
            self.velocitylfPublisher.publish(self.stop)
            self.velocityrfPublisher.publish(self.stop)
            self.velocitylrPublisher.publish(self.stop)
            self.velocityrrPublisher.publish(self.stop)
            self.velocitylmPublisher.publish(self.stop)
            self.velocityrmPublisher.publish(self.stop)

if __name__ == "__main__":
    try:
        system('xset r off')
        control = ControlApp()
        system('xset r on')
    except rospy.ROSInterruptException:
        system('xset r on')
        