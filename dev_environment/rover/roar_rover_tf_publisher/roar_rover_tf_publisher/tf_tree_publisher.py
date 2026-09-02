#!/usr/bin/env python3
"""Publishes only the TF tree of the rover, nothing else.

The joint tree is hardcoded below (extracted once from the rover's URDF/xacro
and verified against a live run of the xacro-based version of this node), so
this node has no dependency on xacro, a URDF, or any other package.

Fixed joints are broadcast once on /tf_static. Non-fixed joints (continuous,
revolute, prismatic, ...) are broadcast on /tf at their zero position on a
timer, since there is no joint_states source wired in. This lets the full
tree (base_footprint -> wheels, arm, sensors, ...) show up without running
robot_state_publisher, joint_state_publisher, Gazebo, or any controllers.
"""
import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

FIXED_JOINT_TYPE = 'fixed'

# (name, type, parent, child, xyz, rpy)
JOINTS = [
    ('base_footprint_joint', 'fixed', 'base_footprint', 'base_link',
     (0.0, 0.0, 0.0), (0.0, 0.0, -1.57079632679)),
    ('rocker_rhs_joint', 'fixed', 'base_link', 'rocker_rhs',
     (0.341, -0.00441357789442404, 0.318458529702179),
     (-1.5707963267949, 0.0269760214941811, -1.5707963267949)),
    ('bogie_rhs_joint', 'fixed', 'rocker_rhs', 'bogie_rhs',
     (-0.224229060097619, 0.165122592734234, -0.0928547999999997),
     (0.0, 0.0, -0.03)),
    ('wheel_rhs_mid_joint', 'continuous', 'bogie_rhs', 'wheel_rhs_mid',
     (0.229224601627686, 0.160798282710695, 0.105843639709434),
     (0.0, 0.0, 3.12607538350607)),
    ('wheel_rhs_front_joint', 'continuous', 'bogie_rhs', 'wheel_rhs_front',
     (-0.229500390800753, 0.160404414952893, 0.104120372913068),
     (0.0, 0.0, -3.14159265358979)),
    ('wheel_rhs_rear_joint', 'continuous', 'rocker_rhs', 'wheel_rhs_rear',
     (0.46794017955705, 0.307105950932406, 0.0129798220506307),
     (0.0, 0.0, 3.14159265358979)),
    ('rocker_lhs_joint', 'fixed', 'base_link', 'rocker_lhs',
     (-0.244499999999996, -0.00517486124912506, 0.320045490177976),
     (1.5707963267949, 0.0269760214941811, -1.5707963267949)),
    ('bogie_lhs_joint', 'fixed', 'rocker_lhs', 'bogie_lhs',
     (-0.224947261786088, -0.16672950973041, 0.00464519999999913),
     (0.0, 0.0, 0.0)),
    ('wheel_lhs_mid_joint', 'continuous', 'bogie_lhs', 'wheel_lhs_mid',
     (0.232061767651008, -0.156675912982079, 0.105848638897971),
     (0.0, 0.0, 3.14159265358979)),
    ('wheel_lhs_front_joint', 'continuous', 'bogie_lhs', 'wheel_lhs_front',
     (-0.226596998316561, -0.164480412414379, 0.105834821721209),
     (0.0, 0.0, 3.14159265358979)),
    ('wheel_lhs_rear_joint', 'continuous', 'rocker_lhs', 'wheel_lhs_rear',
     (0.467221977868636, -0.308712867928559, 0.108765573616487),
     (0.0, 0.0, 3.14159265358979)),
    ('rover_to_arm_mount', 'fixed', 'base_link', 'arm_mount_point',
     (0.0, 0.25, 0.33), (0.0, 0.0, 1.57)),
    ('arm_mount_joint', 'fixed', 'arm_mount_point', 'arm_base_link',
     (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ('arm_joint_0', 'revolute', 'arm_base_link', 'arm_link_1',
     (0.0, 0.0, 0.1), (0.0, 0.0, 0.0)),
    ('arm_joint_1', 'revolute', 'arm_link_1', 'arm_link_2',
     (0.0012324, 0.021637, 0.13613), (1.5708, 0.0, 3.1416)),
    ('arm_joint_2', 'revolute', 'arm_link_2', 'arm_link_3',
     (0.22947, 0.32763, 0.0), (0.0, 0.0, 3.1416)),
    ('arm_joint_3', 'revolute', 'arm_link_3', 'arm_link_4',
     (0.34473, -0.0081531, 0.0), (1.7099, 1.5708, 0.0)),
    ('arm_joint_4', 'revolute', 'arm_link_4', 'arm_link_5',
     (0.0, 0.0, -0.055294), (1.5708, -0.13911, -1.5708)),
    ('arm_joint_5', 'revolute', 'arm_link_5', 'arm_link_6',
     (-0.008596, -0.061393, 0.0), (1.4317, -1.5708, 0.0)),
    ('arm_right_gripper', 'prismatic', 'arm_link_6', 'arm_right_gripper_link',
     (-0.0855, 0.0175, 0.1025), (-1.5708, 0.13911, -1.5708)),
    ('arm_left_gripper', 'prismatic', 'arm_link_6', 'arm_left_gripper_link',
     (0.0165, -0.0175, 0.1025), (-1.5708, -0.13911, 1.5708)),
    ('zed2i_joint', 'fixed', 'base_link', 'zed2i_link',
     (0.0, 0.35, 0.25), (0.0, 0.0, 1.5707963267949)),
    ('zed2i_depth_optical_joint', 'fixed', 'zed2i_link', 'zed2i_depth_optical_frame',
     (0.0, 0.0, 0.0), (-1.5707963267949, 0.0, -1.5707963267949)),
    ('bno055_joint', 'fixed', 'base_link', 'bno055_link',
     (0.0, 0.0, 0.1), (0.0, 0.0, 0.0)),
]


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


class RoverTfTreePublisher(Node):

    def __init__(self):
        super().__init__('rover_tf_tree_publisher')

        self.declare_parameter('publish_rate', 30.0)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = TransformBroadcaster(self)

        static_transforms = [
            self._make_transform(j) for j in JOINTS if j[1] == FIXED_JOINT_TYPE
        ]
        self.dynamic_joints = [j for j in JOINTS if j[1] != FIXED_JOINT_TYPE]

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
        _, _, parent, child, xyz, rpy = joint
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        qx, qy, qz, qw = rpy_to_quaternion(*rpy)
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
