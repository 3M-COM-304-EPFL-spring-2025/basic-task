from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge
import math
import numpy as np
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist

class LaserSubscriber(Node):
    def __init__(self):
        super().__init__('laser_subscriber')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        self.ranges = [0.0] * 640
        self.forward_distance = 1000.0
        self.left_forward_distance = 1000.0
        self.right_forward_distance = 1000.0
        self.left_distance = 1000.0
        self.right_distance = 1000.0
        self.back_distance = 1000.0
        self.accumulated_distance = 0.0
        self.closest_obstacle_distance = 1000.0 # closest obstacle distance forward (0-180 degrees)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.ranges = msg.ranges
        self.left_distance = msg.ranges[320]
        self.left_forward_distance = msg.ranges[240]
        self.forward_distance = msg.ranges[160]
        self.right_forward_distance = msg.ranges[80]
        self.right_distance = msg.ranges[0]
        self.back_distance = msg.ranges[480]
        self.closest_obstacle_distance = min(msg.ranges[0:320])
        print(f"Left: {self.left_distance:.2f}, Forward: {self.forward_distance:.2f}, Right: {self.right_distance:.2f}, Back: {self.back_distance:.2f}")
        print(f"Closest obstacle distance: {self.closest_obstacle_distance:.2f}")

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

class CameraSubscriber(Node):
    def __init__(self):
        print("Hi")
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.listener_callback, 10)
        print("Okay")
        self.bridge = CvBridge()
        print("Next")
        self.current_frame = None

    def listener_callback(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camera Feed", self.current_frame)
            if cv2.getWindowProperty("Camera Feed", cv2.WND_PROP_VISIBLE) < 1:
                cv2.destroyAllWindows()
                return
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

class PIDController:
    def __init__(self, Kp, Ki, Kd, output_limits=(-1, 1)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limits = output_limits

        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        output = max(self.output_limits[0], min(self.output_limits[1], output))

        self.prev_error = error
        print(output)
        return output
    
def reset_commands(command: Twist) -> Twist:
    """Resets all Twist commands to zero."""
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0
    return command

def classify_semi_circle_direction(img, box_coords, visualize=True):
    """
    Classify occluded ball as left- or right-sided semicircle. Optionally visualize curvature.
    :param img: Full image (BGR)
    :param box_coords: (x1, y1, x2, y2) tuple from YOLO bounding box
    :param visualize: If True, draw an arrow indicating curvature
    :return: 'Left-sided semicircle', 'Right-sided semicircle', or 'Unknown'
    """
    x1, y1, x2, y2 = map(int, box_coords)
    ball_roi = img[y1:y2, x1:x2]

    if ball_roi.size == 0:
        return "unknown"

    # Check for occlusion (bounding box too elongated means occlusion)
    w = x2 - x1
    h = y2 - y1
    aspect_ratio = w / h if h != 0 else 0

    if 0.8 < aspect_ratio < 1.2:
        # Likely a full ball, skip semi-circle classification
        return "whole"

    # Grayscale and Threshold
    gray = cv2.cvtColor(ball_roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Find Contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return "unknown"

    # Use largest contour
    contour = max(contours, key=cv2.contourArea)
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return "unknown"

    cx = int(M["m10"] / M["m00"])  # centroid x in ROI
    width = ball_roi.shape[1]

    if cx < width * 0.5:
        direction = "left"
        arrow_start = (x1 + int(w * 0.7), y1 + int(h * 0.5))
        arrow_end = (x1 + int(w * 0.4), y1 + int(h * 0.5))
    elif cx > width * 0.5:
        direction = "right"
        arrow_start = (x1 + int(w * 0.3), y1 + int(h * 0.5))
        arrow_end = (x1 + int(w * 0.6), y1 + int(h * 0.5))
    else:
        direction = "unknown"
        arrow_start = None
        arrow_end = None

    # Visualization
    if visualize and arrow_start and arrow_end:
        cv2.arrowedLine(img, arrow_start, arrow_end, (0, 255, 0), 2, tipLength=0.4)
        cv2.putText(img, direction, (x1, y2 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    return direction

def detect_ball(camera_subscriber: CameraSubscriber, model, frame) -> tuple[bool, str, float, float]:
    """Detects ball using YOLO model."""
    rclpy.spin_once(camera_subscriber)

    if camera_subscriber.current_frame is None:
        camera_subscriber.get_logger().info("No image received yet.")
        return False, None, 0.0, 0.0

    results = model(frame)

    for box in results[0].boxes:
        box_coords = box.xyxy[0].tolist()
        x1, y1, x2, y2 = box_coords
        area = (x2 - x1) * (y2 - y1)
        box_center = (box_coords[0] + box_coords[2]) / 2

        class_id = int(box.cls)
        annotated_frame = results[0].plot()
        cv2.imshow("Annotated Frame", annotated_frame)
        cv2.waitKey(1)

        if class_id == 32:  # Adjust according to your ball class ID
            return True, classify_semi_circle_direction(frame, box_coords), area, box_center

    return False, None, 0.0, 0.0

def main(args=None):
    print("Starting capture")
    rclpy.init(args=args)
    subscriber = LaserSubscriber()
    publisher = CmdVelPublisher()
    camera_subscriber = CameraSubscriber()
    command = Twist()

    model = YOLO("yolov8x.pt")

    import time
    last_time = time.time()
    # TODO: adjust parameters  
    pid = PIDController(Kp=0.005, Ki=0.0001, Kd=0.002, output_limits=(-1, 1))
    time_since_last_detection = 0  # For tracking ball lost time
    max_lost_time = 2.0  # Stop if ball is undetected for 2 seconds
    threshold_area = 4000  # Example threshold for bounding box area
    safe_distance = 0.5  # Example safe distance for obstacles

    try:
        while rclpy.ok():
            print("Waiting for camera frame...")
            rclpy.spin_once(camera_subscriber)
            frame = camera_subscriber.current_frame
            print("Frame received")

            if frame is None:
                print("No frame received. Waiting...")
                time.sleep(0.1)
                continue

            ball_detected, direction, bounding_box_area, box_center = detect_ball(camera_subscriber, model, frame)
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            if ball_detected:
                print("Ball detected!")
                time_since_last_detection = 0

                if bounding_box_area > threshold_area:
                    print("Ball is close enough. Stopping.")
                    command = reset_commands(command)
                elif direction == "right":
                    print("Moving diagonal right to avoid obstacle")
                    frame_center = frame.shape[1] / 4
                    error = frame_center - box_center
                    pid_output = pid.compute(error, dt)
                    command.linear.x = 0.5
                    command.angular.z = pid_output
                elif direction == "left":
                    print("Moving diagonal left to avoid obstacle")
                    frame_center = frame.shape[1] *3 / 4
                    error = frame_center - box_center
                    pid_output = pid.compute(error, dt)
                    command.linear.x = 0.5
                    command.angular.z = pid_output
                else:
                    print("Moving forward towards ball")
                    frame_center = frame.shape[1] / 2
                    error = frame_center - box_center
                    pid_output = pid.compute(error, dt)
                    command.angular.z = pid_output
                    command.linear.x = 0.5

            else:
                time_since_last_detection += time.time() - last_time
                if time_since_last_detection > max_lost_time:
                    print("Ball lost for too long. Stopping.")
                    command = reset_commands(command)
            if subscriber.forward_distance < safe_distance:
                print("Obstacle detected! Stopping.")
                command = reset_commands(command)

            publisher.publisher_.publish(command)

            # Optional: Add a small delay to control loop frequency
            time.sleep(5)

            # PID control logic (if needed for fine adjustments)

            # Example PID usage (if applicable):
            # error = desired_value - current_value
            # pid_output = pid.compute(error, dt)
            # Adjust command.linear.x or command.angular.z based on pid_output

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        reset_commands(command)
        publisher.publisher_.publish(command)
        rclpy.shutdown()

main()
