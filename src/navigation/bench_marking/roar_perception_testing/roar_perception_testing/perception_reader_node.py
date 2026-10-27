#!/usr/bin/env python3
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray


class PerceptionReaderNode(Node):
    """Reads obstacle and ArUco Detection3DArray topics, validates and republishes them."""

    def __init__(self):
        super().__init__('perception_reader_node')
        self.declare_parameter('obstacle_input_topic', '/detection/lidar_clusters')
        self.declare_parameter('obstacle_output_topic', '/benchmark/perception/raw_objects')
        self.declare_parameter('aruco_input_topic', '/detection/aruco_3d')
        self.declare_parameter('aruco_output_topic', '/benchmark/perception/raw_aruco')

        self.obstacle_frames = 0
        self.obstacle_objects = 0
        self.aruco_frames = 0
        self.aruco_objects = 0
        self.invalid = 0

        obstacle_input = str(self.get_parameter('obstacle_input_topic').value)
        obstacle_output = str(self.get_parameter('obstacle_output_topic').value)
        aruco_input = str(self.get_parameter('aruco_input_topic').value)
        aruco_output = str(self.get_parameter('aruco_output_topic').value)

        self.obstacle_pub = self.create_publisher(
            Detection3DArray, obstacle_output, qos_profile_sensor_data)
        self.aruco_pub = self.create_publisher(
            Detection3DArray, aruco_output, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(
            String, '/benchmark/perception/reader_status', 10)

        self.obstacle_sub = self.create_subscription(
            Detection3DArray, obstacle_input, self.obstacle_callback,
            qos_profile_sensor_data)
        self.aruco_sub = self.create_subscription(
            Detection3DArray, aruco_input, self.aruco_callback,
            qos_profile_sensor_data)

        self.get_logger().info(
            f'Obstacles: {obstacle_input} -> {obstacle_output}')
        self.get_logger().info(
            f'ArUco: {aruco_input} -> {aruco_output}')

    @staticmethod
    def valid_detection(det) -> bool:
        values = [
            det.bbox.center.position.x,
            det.bbox.center.position.y,
            det.bbox.center.position.z,
            det.bbox.size.x,
            det.bbox.size.y,
            det.bbox.size.z,
        ]
        return all(math.isfinite(v) for v in values) and all(v >= 0 for v in values[3:])

    def clean_message(self, msg: Detection3DArray) -> Detection3DArray:
        clean = Detection3DArray()
        clean.header = msg.header
        for det in msg.detections:
            if self.valid_detection(det):
                clean.detections.append(det)
            else:
                self.invalid += 1
        return clean

    def obstacle_callback(self, msg: Detection3DArray):
        clean = self.clean_message(msg)
        self.obstacle_frames += 1
        self.obstacle_objects += len(clean.detections)
        self.obstacle_pub.publish(clean)
        self.publish_status(msg.header.frame_id)

    def aruco_callback(self, msg: Detection3DArray):
        clean = self.clean_message(msg)
        self.aruco_frames += 1
        self.aruco_objects += len(clean.detections)
        self.aruco_pub.publish(clean)
        self.publish_status(msg.header.frame_id)

    def publish_status(self, frame_id: str):
        status = String()
        status.data = json.dumps({
            'state': 'running',
            'obstacle_frames': self.obstacle_frames,
            'obstacle_objects': self.obstacle_objects,
            'aruco_frames': self.aruco_frames,
            'aruco_objects': self.aruco_objects,
            'invalid_objects': self.invalid,
            'last_frame_id': frame_id,
        })
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
