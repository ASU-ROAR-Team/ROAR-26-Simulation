#!/usr/bin/env python3
"""Obstacle scenario generator node for benchmarking.

Publishes a `nav_msgs/OccupancyGrid` on `/local_costmap/costmap`.
Supports several built-in scenarios and repeatable randomness via seed.
"""

import math
import time
import random
from typing import Tuple

# Make numpy optional so the node can run in minimal environments; fallback to list-based grids.
try:
    import numpy as np
except Exception:
    np = None

import traceback
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray


class ObstacleGeneratorNode(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_generator')

        # Parameters
        self.declare_parameter('scenario', '')
        self.declare_parameter('seed', 0)
        self.declare_parameter('map_width', 169)
        self.declare_parameter('map_height', 114)
        self.declare_parameter('resolution', 0.25)
        self.declare_parameter('origin_x', -21.125)
        self.declare_parameter('origin_y', -14.25)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('unknown_probability', 0.0)

        self.scenario = str(self.get_parameter('scenario').value or '').lower()
        self.seed = int(self.get_parameter('seed').value)
        self.width = int(self.get_parameter('map_width').value)
        self.height = int(self.get_parameter('map_height').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.unknown_prob = float(self.get_parameter('unknown_probability').value)

        self.get_logger().info(
            f"ObstacleGenerator parameters: scenario={self.scenario}, seed={self.seed}, "
            f"map_width={self.width}, map_height={self.height}, resolution={self.resolution}, "
            f"origin_x={self.origin_x}, origin_y={self.origin_y}, frame_id={self.frame_id}, "
            f"publish_rate_hz={self.publish_rate}, unknown_probability={self.unknown_prob}"
        )

        random.seed(self.seed)
        if np is not None:
            np.random.seed(self.seed)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub = self.create_publisher(OccupancyGrid, '/local_costmap/costmap', qos)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/obstacle_markers',
            10
        )

        # Build initial map
        self.grid = self.generate_map()
        self.get_logger().info(f"ObstacleGenerator: scenario={self.scenario} size={self.width}x{self.height} res={self.resolution}")

        occupied_indices = []
        if np is not None:
            occ_mask = (self.grid == 100)
            occ_coords = list(zip(*np.nonzero(occ_mask)))
            occupied_indices = occ_coords
            occ_count = len(occ_coords)
        else:
            occ_coords = [(y, x) for y in range(self.height) for x in range(self.width) if self.grid[y][x] == 100]
            occupied_indices = occ_coords
            occ_count = len(occ_coords)

        self.get_logger().info(f"ObstacleGenerator: occupied_cells={occ_count}")
        if occ_count > 0:
            sample = occupied_indices[:20]
            sample_world = [(
                float(self.origin_x + (x + 0.5) * self.resolution),
                float(self.origin_y + (y + 0.5) * self.resolution)
            ) for (y, x) in sample]
            self.get_logger().info(f"ObstacleGenerator: sample_occupied_cells={sample_world}")

        out_png = f"/tmp/obstacle_{self.scenario}_{self.seed}.png"
        saved = False
        # Prefer matplotlib if available
        try:
            import matplotlib.pyplot as plt
            if np is not None:
                img = np.zeros((self.height, self.width), dtype=np.uint8)
                img[:] = 255
                img[occ_mask] = 0
            else:
                import numpy as _np
                img = _np.zeros((self.height, self.width), dtype=_np.uint8)
                for (y, x) in occupied_indices:
                    img[y, x] = 0
            plt.imsave(out_png, img, cmap='gray')
            saved = True
        except Exception:
            try:
                from PIL import Image
                if np is not None:
                    img = (255 - (self.grid == 100).astype(np.uint8) * 255)
                    pil_img = Image.fromarray(img)
                else:
                    img = [[255 - (1 if self.grid[y][x] == 100 else 0) * 255 for x in range(self.width)] for y in range(self.height)]
                    pil_img = Image.new('L', (self.width, self.height))
                    pil_img.putdata([pixel for row in img for pixel in row])
                pil_img.save(out_png)
                saved = True
            except Exception:
                saved = False

        if saved:
            self.get_logger().info(f"ObstacleGenerator: saved PNG snapshot to {out_png}")
        else:
            self.get_logger().warning("ObstacleGenerator: could not save PNG snapshot (matplotlib/PIL missing?)")

        # Timer to (re)publish map periodically so late subscribers receive it
        self.timer = self.create_timer(1.0 / max(0.1, self.publish_rate), self._publish_map)

    def generate_map(self) -> np.ndarray:
        # 0 free, 100 occupied, -1 unknown
        if np is not None:
            grid = np.zeros((self.height, self.width), dtype=np.int8)

            # Add optional unknowns
            if self.unknown_prob > 0.0:
                mask = np.random.rand(self.height, self.width) < self.unknown_prob
                grid[mask] = -1

            if self.scenario in ('', 'empty'):
                # keep empty
                return grid
        else:
            # Fallback: pure python list-of-lists
            grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
            if self.unknown_prob > 0.0:
                for y in range(self.height):
                    for x in range(self.width):
                        if random.random() < self.unknown_prob:
                            grid[y][x] = -1
            if self.scenario in ('', 'empty'):
                return grid

        cx = self.width // 2
        cy = self.height // 2

        if self.scenario == 'single_obstacle':
            # vertical bar crossing the middle
            thickness = max(1, int(1.0 / self.resolution))
            x0 = cx - thickness // 2
            if np is not None:
                grid[cy - 10:cy + 10, x0:x0 + thickness] = 100
                return grid
            else:
                for yy in range(cy - 10, cy + 10):
                    if 0 <= yy < self.height:
                        for xx in range(x0, x0 + thickness):
                            if 0 <= xx < self.width:
                                grid[yy][xx] = 100
                return grid

        if self.scenario == 'blocking_obstacle':
            # realistic obstacle block
            block_h = max(2, int(0.6 / self.resolution))   # 0.6 meters deep
            block_w = max(4, int(1.5 / self.resolution))   # 1.5 meters wide

            x0 = cx - block_w // 2
            y0 = cy - block_h // 2

            gap_w = max(2, int(0.8 / self.resolution))     # 0.8 meter gap
            gap_x = cx - gap_w // 2

            self.block_w = block_w
            self.cx = cx
            if np is not None:
                grid[y0:y0 + block_h, x0:x0 + block_w] = 100
                grid[y0 + block_h//2 - 1:y0 + block_h//2 + 1,
                     gap_x:gap_x + gap_w] = 0
                return grid
            else:
                for yy in range(y0, y0 + block_h):
                    if 0 <= yy < self.height:
                        for xx in range(x0, x0 + block_w):
                            if 0 <= xx < self.width:
                                grid[yy][xx] = 100
                gy = y0 + block_h//2
                for yy in range(gy - 1, gy + 1):
                    if 0 <= yy < self.height:
                        for xx in range(gap_x, gap_x + gap_w):
                            if 0 <= xx < self.width:
                                grid[yy][xx] = 0
                return grid

        if self.scenario == 'random_obstacles':
            density = 0.08  # 8% occupied
            if np is not None:
                mask = np.random.rand(self.height, self.width) < density
                grid[mask] = 100
                return grid
            else:
                for yy in range(self.height):
                    for xx in range(self.width):
                        if random.random() < density:
                            grid[yy][xx] = 100
                return grid

        if self.scenario == 'corridor':
            # create vertical walls leaving a narrow corridor along center
            wall_thickness = max(1, int(0.5 / self.resolution))
            corridor_width_m = 1.0
            corridor_width_cells = max(2, int(corridor_width_m / self.resolution))
            left = cx - corridor_width_cells // 2 - 15
            right = cx + corridor_width_cells // 2 + 15
            if np is not None:
                grid[:, :left] = 100
                grid[:, right:] = 100
                return grid
            else:
                for yy in range(self.height):
                    for xx in range(self.width):
                        if xx < left or xx >= right:
                            grid[yy][xx] = 100
                return grid

        # unknown scenario name -> fall back to empty
        self.get_logger().warning(f"Unknown scenario '{self.scenario}' - generating empty map")
        return grid

    def _publish_markers(self):
        markers = MarkerArray()

        marker = Marker()

        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Visualize blocking obstacle region
        if self.scenario == "blocking_obstacle":

            block_h = max(2, int(0.6 / self.resolution))

            marker.pose.position.x = (
                self.origin_x +
                (self.cx + 0.5) * self.resolution
            )

            marker.pose.position.y = (
                self.origin_y +
                (self.height // 2) * self.resolution
            )

            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0

            marker.scale.x = self.block_w * self.resolution
            marker.scale.y = block_h * self.resolution
            marker.scale.z = 0.1

        else:
            # fallback: visualize occupied area bounding box
            occupied = []

            for y in range(self.height):
                for x in range(self.width):
                    if int(self.grid[y][x]) >= 50:
                        occupied.append((x, y))

            if not occupied:
                return

            xs = [p[0] for p in occupied]
            ys = [p[1] for p in occupied]

            marker.pose.position.x = (
                self.origin_x +
                ((min(xs)+max(xs))/2 + 0.5) * self.resolution
            )

            marker.pose.position.y = (
                self.origin_y +
                ((min(ys)+max(ys))/2 + 0.5) * self.resolution
            )

            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0

            marker.scale.x = (max(xs)-min(xs)+1) * self.resolution
            marker.scale.y = (max(ys)-min(ys)+1) * self.resolution
            marker.scale.z = 0.1


        marker.color.a = 0.8
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        markers.markers.append(marker)

        self.marker_pub.publish(markers)

    def _publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        info = MapMetaData()
        info.resolution = float(self.resolution)
        info.width = int(self.width)
        info.height = int(self.height)
        info.origin = Pose()
        info.origin.position.x = float(self.origin_x)
        info.origin.position.y = float(self.origin_y)
        info.origin.position.z = 0.0
        info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        msg.info = info

        # Flatten grid into a list of ints (-1..100)
        if np is not None and hasattr(self.grid, 'flatten'):
            flat = self.grid.flatten().tolist()
        else:
            # python list-of-lists
            flat = [int(cell) for row in self.grid for cell in row]
        msg.data = [int(x) for x in flat]

        self.pub.publish(msg)
        self._publish_markers()
        try:
            occ = sum(1 for v in msg.data if int(v) >= 50)
            self.get_logger().info(f'Published occupancy grid with {occ} occupied cells')
        except Exception:
            self.get_logger().debug('Published occupancy grid')


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ObstacleGeneratorNode()
        # Startup confirmation for debugging
        try:
            print('ObstacleGenerator started successfully')
            node.get_logger().info('ObstacleGenerator started successfully')
        except Exception:
            pass
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
    except Exception as e:
        # Write full traceback to a file for easier diagnosis in launch logs
        tb = traceback.format_exc()
        try:
            with open('/tmp/obstacle_generator_error.log', 'w') as f:
                f.write(tb)
        except Exception:
            pass
        print('ObstacleGenerator crashed. Traceback written to /tmp/obstacle_generator_error.log', file=sys.stderr)
        print(tb, file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()