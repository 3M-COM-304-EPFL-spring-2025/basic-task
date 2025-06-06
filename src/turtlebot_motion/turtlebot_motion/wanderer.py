# Written by Enrique Fernández-Laguilhoat Sánchez-Biezma
import time
from random import random
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# Distance from wall threshold
DISTANCE_FROM_WALL = 1.0


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
        self.size=0

    def listener_callback(self, msg):
        self.ranges = msg.ranges
        self.size=len(msg.ranges)
        self.left_distance = msg.ranges[self.size//2]
        self.left_forward_distance = msg.ranges[3*self.size//8]
        self.forward_distance = msg.ranges[self.size//4]
        self.right_forward_distance = msg.ranges[self.size//8]
        self.right_distance = msg.ranges[0]
        self.back_distance = msg.ranges[3*self.size//4]
        self.closest_obstacle_distance = min(msg.ranges[0:self.size//2])
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
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")


def reset_commands(command: Twist) -> Twist:
    """Resets all Twist commands to zero."""
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0
    return command


def check_ranges(subscriber: LaserSubscriber) -> tuple[bool, float]:
    """Check if path ahead is clear."""
    rclpy.spin_once(subscriber)
    readings = [subscriber.forward_distance, subscriber.left_forward_distance, subscriber.right_forward_distance]
    min_distance = min(readings)

    if subscriber.forward_distance < DISTANCE_FROM_WALL or subscriber.closest_obstacle_distance < DISTANCE_FROM_WALL/2:
        subscriber.get_logger().info("Obstacle detected ahead.")
        return False, min_distance

    if (subscriber.right_forward_distance > DISTANCE_FROM_WALL and
            subscriber.left_forward_distance > DISTANCE_FROM_WALL):
        return True, min_distance

    if (subscriber.left_forward_distance > subscriber.left_distance or
            subscriber.right_forward_distance > subscriber.right_distance):
        return True, min_distance

    subscriber.get_logger().info("Obstacle detected nearby.")
    return False, min_distance


def detect_ball(camera_subscriber: CameraSubscriber, model: YOLO) -> bool:
    """Detects ball using YOLO model."""
    rclpy.spin_once(camera_subscriber)

    if camera_subscriber.current_frame is None:
        camera_subscriber.get_logger().info("No image received yet.")
        return False

    frame = camera_subscriber.current_frame
    results = model(frame)

    for box in results[0].boxes:
        class_id = int(box.cls)
        annotated_frame = results[0].plot()
        cv2.imshow("Annotated Frame", annotated_frame)
        cv2.waitKey(1)

        if class_id == 24:  # Adjust according to your ball class ID
            return True

    return False


def go_forward_until_obstacle(subscriber: LaserSubscriber, publisher: CmdVelPublisher, command: Twist, camera_subscriber: CameraSubscriber, model: YOLO):
    """Moves robot forward until obstacle detected."""
    command = reset_commands(command)

    while check_ranges(subscriber)[0]:
        rclpy.spin_once(subscriber)
        speed = min(check_ranges(subscriber)[1] / 2.5, 2.2)  # Max speed capped
        command.linear.x = speed
        publisher.get_logger().info(f"Moving forward at {round(speed, 2)} m/s")
        publisher.publisher_.publish(command)
        subscriber.accumulated_distance += speed * 0.1  # Assuming 10Hz loop rate
        print(f"Accumulated distance: {subscriber.accumulated_distance:.2f} m")

        if subscriber.accumulated_distance > 2.5:
            # Reset accumulated distance if too far
            subscriber.accumulated_distance = 0.0
            publisher.get_logger().info("Resetting accumulated distance.")
            # Check for ball detection
            if spin_detect_ball(subscriber, publisher, command, camera_subscriber, model):
                print("Ball detected! Taking action...")
                return True

    publisher.publisher_.publish(reset_commands(command))
    return False


def rotate_until_clear(subscriber: LaserSubscriber, publisher: CmdVelPublisher, command: Twist):
    """Rotates robot until path ahead is clear."""
    command = reset_commands(command)
    rclpy.spin_once(subscriber)

    if subscriber.left_forward_distance < subscriber.right_forward_distance:
        # Prefer turning right
        while (subscriber.left_forward_distance < subscriber.left_distance or
               subscriber.forward_distance < DISTANCE_FROM_WALL or subscriber.closest_obstacle_distance < DISTANCE_FROM_WALL/2):
            rclpy.spin_once(subscriber)
            command.angular.z = -1.1 + random() * 0.3
            publisher.publisher_.publish(command)
            publisher.get_logger().info("Rotating right...")
    else:
        # Prefer turning left
        while (subscriber.right_forward_distance < subscriber.right_distance or
               subscriber.forward_distance < DISTANCE_FROM_WALL or subscriber.closest_obstacle_distance < DISTANCE_FROM_WALL/2):
            rclpy.spin_once(subscriber)
            command.angular.z = 1.1 - random() * 0.3
            publisher.publisher_.publish(command)
            publisher.get_logger().info("Rotating left...")

    publisher.publisher_.publish(reset_commands(command))
    subscriber.get_logger().info("Path is now clear.")


def spin_detect_ball(subscriber, publisher, command, camera_subscriber, model):
    """
    Makes the robot spin 360 degrees, stopping every 60 degrees to check for a ball using YOLO.
    """
    command = reset_commands(command)
    publisher.get_logger().info("Starting 360° spin with 6 detection checks...")

    ball_detected = False

    angular_speed = 2.0  # radians per second
    angle_per_step = math.pi / 6  # 60 degrees = π/3 radians
    spin_time_per_step = angle_per_step / abs(angular_speed)  # time to rotate 60 degrees

    for step in range(6):
        publisher.get_logger().info(f"Step {step + 1} of 6: Rotating {math.degrees(angle_per_step)} degrees...")

        # Start rotating
        command = reset_commands(command)
        command.angular.z = angular_speed
        publisher.publisher_.publish(command)

        # Wait exactly the needed time
        start_time = time.time()
        while time.time() - start_time < 2*spin_time_per_step:
            rclpy.spin_once(subscriber)
            rclpy.spin_once(camera_subscriber)

        # Stop rotation
        command = reset_commands(command)
        publisher.publisher_.publish(command)
        time.sleep(2)  # short pause for stability

        publisher.get_logger().info(f"Checking for ball at step {step + 1}...")

        # Perform YOLO detection
        if detect_ball(camera_subscriber, model):
            ball_detected = True
            publisher.get_logger().info("Ball detected!")
            break
        else:
            publisher.get_logger().info("No ball detected.")

    command = reset_commands(command)
    publisher.publisher_.publish(command)
    publisher.get_logger().info("Finished 360° spin.")

    return ball_detected



def main(args=None):
    print("Starting robot wanderer node...")
    rclpy.init(args=args)

    subscriber = LaserSubscriber()
    publisher = CmdVelPublisher()
    camera_subscriber = CameraSubscriber()
    command = Twist()

    model = YOLO("yolov8x.pt")  # Replace with your custom model if necessary

    print("Waiting for camera and laser scan data...")
    #rclpy.spin_once(camera_subscriber)
    print("Camera data received.")

    try:
        while rclpy.ok():
            print("yallah")
            if go_forward_until_obstacle(subscriber, publisher, command, camera_subscriber, model):
                print("Ball detected! Taking action...")
                break
            rotate_until_clear(subscriber, publisher, command)

    finally:
        subscriber.destroy_node()
        publisher.destroy_node()
        camera_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
