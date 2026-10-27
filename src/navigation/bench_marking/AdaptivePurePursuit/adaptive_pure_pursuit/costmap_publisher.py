#!/usr/bin/env python3
"""Publish a static costmap from costmap.npz as a nav_msgs/OccupancyGrid.

This is a minimal publisher to provide `/local_costmap/costmap` for MPPI's
CostCritic/ObstaclesCritic in benchmarking mode. It is intentionally simple
and does not implement dynamic sensor layers.
"""

import os
import time
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from ament_index_python.packages import get_package_share_directory


class CostmapPublisher(Node):
    def __init__(self) -> None:
        super().__init__('local_costmap_publisher')

        # Parameters
        pkg_share = get_package_share_directory('adaptive_pure_pursuit')
        default_npz = os.path.join(pkg_share, 'costmap.npz')
        self.declare_parameter('npz_file', default_npz)
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width_m', 10.0)
        self.declare_parameter('height_m', 10.0)
        self.declare_parameter('origin_x', -5.0)
        self.declare_parameter('origin_y', -5.0)

        self.npz_file = self.get_parameter('npz_file').value
        self.rate = float(self.get_parameter('publish_rate_hz').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.resolution = float(self.get_parameter('resolution').value)
        self.width_m = float(self.get_parameter('width_m').value)
        self.height_m = float(self.get_parameter('height_m').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)

        self.width = int(round(self.width_m / self.resolution))
        self.height = int(round(self.height_m / self.resolution))

        self.get_logger().info(f'CostmapPublisher: loading {self.npz_file}')
        self.costgrid = self._load_npz(self.npz_file)
        if self.costgrid is None:
            # create empty free grid
            self.get_logger().warn('Failed to load costmap array; publishing empty free grid')
            self.costgrid = np.zeros((self.height, self.width), dtype=np.int8)

        # Ensure shape matches expected (height, width)
        if self.costgrid.shape != (self.height, self.width):
            self.get_logger().warn(f'Costmap shape {self.costgrid.shape} does not match expected {(self.height, self.width)}; resizing')
            # Resize or pad/truncate as needed
            grid = np.zeros((self.height, self.width), dtype=np.int8)
            h = min(self.height, self.costgrid.shape[0])
            w = min(self.width, self.costgrid.shape[1])
            grid[0:h, 0:w] = self.costgrid[0:h, 0:w]
            self.costgrid = grid

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(OccupancyGrid, '/local_costmap/costmap', qos)

        self.timer = self.create_timer(1.0 / max(1e-3, self.rate), self.timer_callback)

    def _load_npz(self, path: str) -> Any:
        try:
            archive = np.load(path)
            # Pick first array in the archive
            if isinstance(archive, np.lib.npyio.NpzFile):
                keys = list(archive.keys())
                if not keys:
                    return None
                arr = archive[keys[0]]
            else:
                arr = archive

            arr = np.array(arr)
            # Normalize to 0..100 occupancy. Treat non-zero as obstacle
            if arr.dtype != np.int8 and arr.dtype != np.int16 and arr.dtype != np.int32:
                arr = (arr > 0).astype(np.int8) * 100
            else:
                # map any non-zero to 100
                arr = (arr != 0).astype(np.int8) * 100

            # If arr is (W,H) or (H,W) try to coerce to (H,W)
            if arr.ndim == 2:
                if arr.shape == (self.height, self.width):
                    return arr
                if arr.shape == (self.width, self.height):
                    return arr.T
                # else will be resized later
                return arr
            else:
                return None
        except Exception as e:
            self.get_logger().error(f'Error loading npz costmap: {e}')
            return None

    def timer_callback(self) -> None:
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.width)
        msg.info.height = int(self.height)
        msg.info.origin = Pose()
        msg.info.origin.position = Point(x=float(self.origin_x), y=float(self.origin_y), z=0.0)
        msg.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Flatten row-major (row0..rowN) as list of int8 in row-major order
        # OccupancyGrid expects int8 with -1 unknown, 0..100 occupancy
        flat = self.costgrid.flatten(order='C').tolist()
        msg.data = [int(x) for x in flat]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
