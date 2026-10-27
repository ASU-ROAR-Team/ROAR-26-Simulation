#!/usr/bin/env python3
import json

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray


class PerceptionTFNode(Node):
    """Transforms obstacle and ArUco Detection3DArray messages to one target frame."""

    def __init__(self):
        super().__init__('perception_tf_node')
        self.declare_parameter('obstacle_input_topic', '/benchmark/perception/raw_objects')
        self.declare_parameter('obstacle_output_topic', '/benchmark/perception/world_objects')
        self.declare_parameter('aruco_input_topic', '/benchmark/perception/raw_aruco')
        self.declare_parameter('aruco_output_topic', '/benchmark/perception/world_aruco')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('tf_timeout_sec', 0.25)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.timeout = float(self.get_parameter('tf_timeout_sec').value)
        self.transformed_obstacles = 0
        self.transformed_aruco = 0
        self.failed_obstacles = 0
        self.failed_aruco = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        obstacle_in = str(self.get_parameter('obstacle_input_topic').value)
        obstacle_out = str(self.get_parameter('obstacle_output_topic').value)
        aruco_in = str(self.get_parameter('aruco_input_topic').value)
        aruco_out = str(self.get_parameter('aruco_output_topic').value)

        self.obstacle_pub = self.create_publisher(
            Detection3DArray, obstacle_out, qos_profile_sensor_data)
        self.aruco_pub = self.create_publisher(
            Detection3DArray, aruco_out, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(
            String, '/benchmark/perception/tf_status', 10)

        self.obstacle_sub = self.create_subscription(
            Detection3DArray, obstacle_in,
            lambda msg: self.transform_callback(msg, 'obstacle'),
            qos_profile_sensor_data)
        self.aruco_sub = self.create_subscription(
            Detection3DArray, aruco_in,
            lambda msg: self.transform_callback(msg, 'aruco'),
            qos_profile_sensor_data)

        self.get_logger().info(
            f'TF target frame: "{self.target_frame}"')
        self.get_logger().info(f'Obstacles: {obstacle_in} -> {obstacle_out}')
        self.get_logger().info(f'ArUco: {aruco_in} -> {aruco_out}')

    def transform_callback(self, msg: Detection3DArray, stream: str):
        if not msg.header.frame_id:
            self.record_failure(stream, len(msg.detections), 'Empty source frame')
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=self.timeout),
            )
        except TransformException as exc:
            self.record_failure(stream, len(msg.detections), str(exc))
            self.get_logger().warning(f'{stream} TF failed: {exc}')
            return

        output = Detection3DArray()
        output.header = msg.header
        output.header.frame_id = self.target_frame

        for det in msg.detections:
            source_pose = PoseStamped()
            source_pose.header = msg.header
            source_pose.pose = det.bbox.center
            transformed_pose = do_transform_pose(source_pose.pose, transform)

            out_det = Detection3D()
            out_det.header = output.header
            out_det.bbox.center = transformed_pose
            out_det.bbox.size = det.bbox.size
            out_det.results = det.results
            output.detections.append(out_det)

        if stream == 'obstacle':
            self.transformed_obstacles += len(output.detections)
            self.obstacle_pub.publish(output)
        else:
            self.transformed_aruco += len(output.detections)
            self.aruco_pub.publish(output)
        self.publish_status(msg.header.frame_id, '')

    def record_failure(self, stream: str, count: int, error: str):
        if stream == 'obstacle':
            self.failed_obstacles += count
        else:
            self.failed_aruco += count
        self.publish_status('', error)

    def publish_status(self, source_frame: str, error: str):
        status = String()
        status.data = json.dumps({
            'state': 'running',
            'source_frame': source_frame,
            'target_frame': self.target_frame,
            'transformed_obstacles': self.transformed_obstacles,
            'failed_obstacles': self.failed_obstacles,
            'transformed_aruco': self.transformed_aruco,
            'failed_aruco': self.failed_aruco,
            'last_error': error,
        })
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionTFNode()
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
