
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
import tf_transformations
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy

class VisualCoverageMapper(Node):
    def __init__(self):
        super().__init__('visual_coverage_mapper')

        # Parameters
        self.declare_parameter('fov_deg', 60.0)
        self.declare_parameter('max_range', 3.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('ray_count', 30)

        self.fov_deg = self.get_parameter('fov_deg').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.ray_count = self.get_parameter('ray_count').get_parameter_value().integer_value

        self.map = None
        self.map_info = None

        self.coverage_map = None

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.coverage_pub = self.create_publisher(OccupancyGrid, '/visual_coverage_map', 10)

        self.get_logger().info('Visual Coverage Mapper Node Initialized.')

    def map_callback(self, msg):
        new_map_info = msg.info
        new_height, new_width = new_map_info.height, new_map_info.width
        new_origin = new_map_info.origin

        new_map = np.array(msg.data, dtype=np.int8).reshape((new_height, new_width))

        # First time init
        if self.coverage_map is None:
            self.coverage_map = np.zeros((new_height, new_width), dtype=np.uint8)
            self.map = new_map
            self.map_info = new_map_info
            return

        old_height, old_width = self.map_info.height, self.map_info.width
        old_origin = self.map_info.origin

        # If size or origin changed → reallocate coverage_map
        if new_height != old_height or new_width != old_width or new_origin.position.x != old_origin.position.x or new_origin.position.y != old_origin.position.y:
            new_coverage = np.zeros((new_height, new_width), dtype=np.uint8)

            # Compute offset of old map origin in new map
            dx = int(round((old_origin.position.x - new_origin.position.x) / new_map_info.resolution))
            dy = int(round((old_origin.position.y - new_origin.position.y) / new_map_info.resolution))

            # Copy old data into new_coverage
            y_start = max(0, dy)
            x_start = max(0, dx)
            y_end = min(old_height + dy, new_height)
            x_end = min(old_width + dx, new_width)

            old_y_start = max(0, -dy)
            old_x_start = max(0, -dx)
            old_y_end = old_y_start + (y_end - y_start)
            old_x_end = old_x_start + (x_end - x_start)

            new_coverage[y_start:y_end, x_start:x_end] = self.coverage_map[old_y_start:old_y_end, old_x_start:old_x_end]

            self.coverage_map = new_coverage

        # Save map and info
        self.map = new_map
        self.map_info = new_map_info


    def odom_callback(self, msg):
        if self.map is None or self.coverage_map is None:
            return

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])

        self.update_coverage(position.x, position.y, yaw)
        self.publish_coverage_map()

    def is_obstacle(self, i, j):
        if 0 <= i < self.map_info.width and 0 <= j < self.map_info.height:
            return self.map[j, i] > 50
        return True

    def bresenham_line(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            yield x0, y0
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def update_coverage(self, robot_x, robot_y, yaw):
        angles = np.linspace(-self.fov_deg/2, self.fov_deg/2, num=self.ray_count) * np.pi / 180
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        robot_i = int((robot_x - origin_x) / self.resolution)
        robot_j = int((robot_y - origin_y) / self.resolution)

        for angle in angles:
            theta = yaw + angle
            end_x = robot_x + self.max_range * math.cos(theta)
            end_y = robot_y + self.max_range * math.sin(theta)
            end_i = int((end_x - origin_x) / self.resolution)
            end_j = int((end_y - origin_y) / self.resolution)

            for i, j in self.bresenham_line(robot_i, robot_j, end_i, end_j):
                if 0 <= i < self.map_info.width and 0 <= j < self.map_info.height:
                    self.coverage_map[j, i] = 1
                    if self.is_obstacle(i, j):
                        break
                else:
                    break

    def publish_coverage_map(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info = self.map_info
        data = list(self.coverage_map.flatten() * 100)
        # convert to int8
        data = [int(x) for x in data]
        msg.data = data
        print("GJFFKVEF")
        self.coverage_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisualCoverageMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
