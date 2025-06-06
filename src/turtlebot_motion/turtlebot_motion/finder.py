import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
from ultralytics import YOLO
import cv2


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()
        self.current_frame = None

    def listener_callback(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")


class LaserSubscriber(Node):
    def __init__(self):
        super().__init__('laser_subscriber')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.ranges = []
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0

    def listener_callback(self, msg):
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment


class BallFinder(Node):
    def __init__(self):
        super().__init__('ball_finder')
        self.camera_subscriber = CameraSubscriber()
        self.laser_subscriber = LaserSubscriber()
        self.model = YOLO("yolov8x.pt")  # Load YOLO model
        self.ball_distance = None
        self.ball_angle = None

    def process(self):
        # Ensure a camera frame is available
        if self.camera_subscriber.current_frame is None:
            self.get_logger().info("No camera frame available yet.")
            return

        # Ensure laser scan data is available
        if not self.laser_subscriber.ranges:
            self.get_logger().info("No laser scan data available yet.")
            return

        # Detect the ball using YOLO
        results = self.model(self.camera_subscriber.current_frame)
        ball_detected = False
        ball_center_x = None

        for box in results[0].boxes:
            class_id = int(box.cls)
            if class_id == 32:  # Adjust according to your ball class ID
                box_coords = box.xyxy[0].tolist()
                x1, y1, x2, y2 = map(int, box_coords)
                ball_center_x = (x1 + x2) / 2  # Horizontal center of the bounding box
                ball_detected = True
                break

        if not ball_detected:
            self.get_logger().info("No ball detected by YOLO.")
            return

        # Map the ball's horizontal position to a laser scan angle
        frame_width = self.camera_subscriber.current_frame.shape[1]
        angle_range = self.laser_subscriber.angle_max - self.laser_subscriber.angle_min
        ball_angle = self.laser_subscriber.angle_min + (ball_center_x / frame_width) * angle_range

        # Find the corresponding distance in the laser scan data
        angle_increment = self.laser_subscriber.angle_increment
        index = int((ball_angle - self.laser_subscriber.angle_min) / angle_increment)
        if 0 <= index < len(self.laser_subscriber.ranges):
            ball_distance = self.laser_subscriber.ranges[index]
        else:
            self.get_logger().info("Ball angle out of laser scan range.")
            return

        # Update ball distance and angle
        self.ball_distance = ball_distance
        self.ball_angle = ball_angle
        self.get_logger().info(f"Ball detected at distance: {self.ball_distance}, angle: {math.degrees(self.ball_angle)}")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self.camera_subscriber, timeout_sec=0.1)
            rclpy.spin_once(self.laser_subscriber, timeout_sec=0.1)
            self.process()


def main(args=None):
    rclpy.init(args=args)
    ball_finder = BallFinder()
    try:
        ball_finder.run()
    except KeyboardInterrupt:
        pass
    finally:
        ball_finder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()