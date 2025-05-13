import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from ultralytics import YOLO
import cv2

class BallFinder(Node):
    def __init__(self):
        super().__init__('ball_finder')
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.ball_distance = None
        self.ball_angle = None
        self.model = YOLO("yolo-Weights/yolov8x.pt")  # Load YOLO model
        self.camera_frame = None  # Placeholder for the camera frame

    def laser_callback(self, data):
        # Ensure a camera frame is available
        if self.camera_frame is None:
            self.get_logger().info("No camera frame available yet.")
            return

        # Detect the ball using YOLO
        results = self.model(self.camera_frame)
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
        frame_width = self.camera_frame.shape[1]
        angle_range = data.angle_max - data.angle_min
        ball_angle = data.angle_min + (ball_center_x / frame_width) * angle_range

        # Find the corresponding distance in the laser scan data
        angle_increment = data.angle_increment
        index = int((ball_angle - data.angle_min) / angle_increment)
        if 0 <= index < len(data.ranges):
            ball_distance = data.ranges[index]
        else:
            self.get_logger().info("Ball angle out of laser scan range.")
            return

        # Update ball distance and angle
        self.ball_distance = ball_distance
        self.ball_angle = ball_angle
        self.get_logger().info(f"Ball detected at distance: {self.ball_distance}, angle: {math.degrees(self.ball_angle)}")

    def process_camera_frame(self, frame):
        """Update the current camera frame."""
        self.camera_frame = frame

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    ball_finder = BallFinder()

    # Simulate camera input (replace with actual camera feed in practice)
    cap = cv2.VideoCapture(0)  # Open the default camera
    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                continue

            # Process the camera frame
            ball_finder.process_camera_frame(frame)

            # Spin the ROS2 node
            rclpy.spin_once(ball_finder, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        ball_finder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()