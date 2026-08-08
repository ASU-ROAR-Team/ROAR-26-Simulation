#!/usr/bin/env python3
"""Publishes only the TF tree of a robot described by a URDF, nothing else.

Fixed joints are broadcast once on /tf_static. Non-fixed joints (continuous,
revolute, prismatic, ...) are broadcast on /tf at their zero position on a
timer, since there is no joint_states source wired in. This lets the full
tree (base_footprint -> wheels, arm, sensors, ...) show up without running
robot_state_publisher, joint_state_publisher, Gazebo, or any controllers.
"""
import math
import sys
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

FIXED_JOINT_TYPE = 'fixed'


def rpy_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def parse_joints(urdf_xml):
    """Extract (name, type, parent, child, xyz, rpy) for every joint in a URDF."""
    root = ET.fromstring(urdf_xml)
    joints = []
    for joint_el in root.findall('joint'):
        parent_el = joint_el.find('parent')
        child_el = joint_el.find('child')
        if parent_el is None or child_el is None:
            continue

        xyz = (0.0, 0.0, 0.0)
        rpy = (0.0, 0.0, 0.0)
        origin_el = joint_el.find('origin')
        if origin_el is not None:
            if origin_el.get('xyz'):
                xyz = tuple(float(v) for v in origin_el.get('xyz').split())
            if origin_el.get('rpy'):
                rpy = tuple(float(v) for v in origin_el.get('rpy').split())

        joints.append({
            'name': joint_el.get('name'),
            'type': joint_el.get('type'),
            'parent': parent_el.get('link'),
            'child': child_el.get('link'),
            'xyz': xyz,
            'rpy': rpy,
        })
    return joints


class RoverTfTreePublisher(Node):

    def __init__(self):
        super().__init__('rover_tf_tree_publisher')

        self.declare_parameter('robot_description', '')
        self.declare_parameter('publish_rate', 30.0)

        robot_description = self.get_parameter(
            'robot_description').get_parameter_value().string_value
        if not robot_description:
            raise RuntimeError(
                "Parameter 'robot_description' is empty. Pass the rover's "
                "URDF/XML (e.g. via `Command(['xacro ', xacro_file])` in a launch file).")

        joints = parse_joints(robot_description)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = TransformBroadcaster(self)

        static_transforms = [
            self._make_transform(j) for j in joints if j['type'] == FIXED_JOINT_TYPE
        ]
        self.dynamic_joints = [j for j in joints if j['type'] != FIXED_JOINT_TYPE]

        if static_transforms:
            self.static_broadcaster.sendTransform(static_transforms)
            self.get_logger().info(f'Published {len(static_transforms)} static transforms.')

        if self.dynamic_joints:
            rate = self.get_parameter('publish_rate').get_parameter_value().double_value
            self.get_logger().info(
                f'Broadcasting {len(self.dynamic_joints)} non-fixed joints at zero '
                f'position, {rate:.1f} Hz (no joint_states input wired in).')
            self.create_timer(1.0 / rate, self._publish_dynamic)
        else:
            self.get_logger().info('No non-fixed joints found; TF tree is fully static.')

    def _make_transform(self, joint):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = joint['parent']
        t.child_frame_id = joint['child']
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = joint['xyz']
        qx, qy, qz, qw = rpy_to_quaternion(*joint['rpy'])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        return t

    def _publish_dynamic(self):
        now = self.get_clock().now().to_msg()
        for j in self.dynamic_joints:
            t = self._make_transform(j)
            t.header.stamp = now
            self.dynamic_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RoverTfTreePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f'[rover_tf_tree_publisher] {exc}', file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
