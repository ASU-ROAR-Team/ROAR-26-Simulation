#!/usr/bin/env python3
import rclpy
import os
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from ros_gz_interfaces.msg import Contacts

class CollisionMonitor(Node):
    def __init__(self):
        super().__init__('collision_monitor')
        self.pub = self.create_publisher(Bool, '/rover/collision_flag', 10)
        
        # We will subscribe to all contact topics dynamically.
        # But we need to define the sensors.
        self.declare_parameter('world_name', 'world1')
        self.declare_parameter('model_name', 'roar_rover')
        self.declare_parameter('obstacle_gt_path', '')
        
        world_name = self.get_parameter('world_name').value
        model_name = self.get_parameter('model_name').value
        obstacle_gt = self.get_parameter('obstacle_gt_path').value

        # Build mapping from Entity Name (e.g., 'Rock_1') to Mesh ID
        self.entity_to_mesh = {}
        if obstacle_gt and os.path.exists(obstacle_gt):
            try:
                data = np.load(obstacle_gt, allow_pickle=True)
                for d in data:
                    self.entity_to_mesh[d['name']] = d['mesh_id']
                self.get_logger().info(f"Loaded {len(self.entity_to_mesh)} rock mesh mappings from {obstacle_gt}")
            except Exception as e:
                self.get_logger().error(f"Failed to load obstacle_data.npy: {e}")

        # Non-collidable meshes as requested by the user
        self.safe_meshes = {5, 6, 7, 8, 9}

        sensors = [
            ('base_link', 'base_link_contact'),
            ('wheel_rhs_front', 'wheel_rhs_front_contact'),
            ('wheel_rhs_mid', 'wheel_rhs_mid_contact'),
            ('wheel_rhs_rear', 'wheel_rhs_rear_contact'),
            ('wheel_lhs_front', 'wheel_lhs_front_contact'),
            ('wheel_lhs_mid', 'wheel_lhs_mid_contact'),
            ('wheel_lhs_rear', 'wheel_lhs_rear_contact'),
        ]

        self.subs = []
        for link, sensor in sensors:
            topic = f'/world/{world_name}/model/{model_name}/link/{link}/sensor/{sensor}/contact'
            sub = self.create_subscription(
                Contacts,
                topic,
                self.contact_callback,
                10
            )
            self.subs.append(sub)
            self.get_logger().info(f"Subscribed to contact topic: {topic}")

        # Timer to reset the flag if no contacts have been received recently
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.last_contact_time = self.get_clock().now()
        self.is_colliding = False

    def contact_callback(self, msg: Contacts):
        if not msg.contact:
            return

        for contact in msg.contact:
            c1 = contact.collision1.name if hasattr(contact.collision1, 'name') else ""
            c2 = contact.collision2.name if hasattr(contact.collision2, 'name') else ""
            
            # Find the obstacle name (e.g., "Rock_5::link::collision")
            obstacle_name = c2 if "roar_rover" in c1 else c1
            
            if "Rock_" in obstacle_name:
                # Extract the base entity name (e.g., "Rock_5")
                entity_name = obstacle_name.split("::")[0]
                
                # Look up the mesh_id for this entity
                mesh_id = self.entity_to_mesh.get(entity_name, -1)
                
                if mesh_id in self.safe_meshes:
                    # It's one of the non-collidable meshes (4,5,6,7,9), ignore it!
                    continue

            # If we get here, it's a valid fatal collision (collidable rock, wall, etc.)
            self.is_colliding = True
            self.last_contact_time = self.get_clock().now()
            
            flag_msg = Bool()
            flag_msg.data = True
            self.pub.publish(flag_msg)
            return  # Trigger alarm immediately

    def timer_callback(self):
        # If we haven't seen a contact in 0.5 seconds, set colliding to False
        now = self.get_clock().now()
        if self.is_colliding and (now - self.last_contact_time).nanoseconds > 500_000_000:
            self.is_colliding = False
            flag_msg = Bool()
            flag_msg.data = False
            self.pub.publish(flag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
