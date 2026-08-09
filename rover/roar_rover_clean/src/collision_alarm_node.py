#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool

class CollisionAlarmNode(Node):
    def __init__(self):
        super().__init__('collision_alarm_node')
        self.subscription = self.create_subscription(
            Contacts,
            '/rover_contact',
            self.contact_callback,
            10)
        self.publisher = self.create_publisher(Bool, '/collision_alarm', 10)
        self.get_logger().info('Collision Alarm Node started. Ignoring ground contacts...')
        
        self.is_colliding = False

    def contact_callback(self, msg):
        alarm_msg = Bool()
        
        current_collision = False
        hit_details = ""
        
        for contact in msg.contacts:
            c1 = contact.collision1.name.lower() if hasattr(contact.collision1, 'name') else str(contact.collision1)
            c2 = contact.collision2.name.lower() if hasattr(contact.collision2, 'name') else str(contact.collision2)
            
            # Ignore the ground / mars_yard terrain
            if 'mars_yard' not in c1 and 'mars_yard' not in c2 and 'ground' not in c1 and 'ground' not in c2 and 'terrain' not in c1 and 'terrain' not in c2:
                current_collision = True
                hit_details = f"{c1} <-> {c2}"
                break
        
        if current_collision:
            alarm_msg.data = True
            self.publisher.publish(alarm_msg)
            
            if not self.is_colliding:
                self.get_logger().warn(f'OBSTACLE COLLISION DETECTED! Details: {hit_details}')
                self.is_colliding = True
        else:
            alarm_msg.data = False
            self.publisher.publish(alarm_msg)
            
            if self.is_colliding:
                self.get_logger().info('Rover is clear of obstacles.')
                self.is_colliding = False

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAlarmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
