import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np

from irobot_create_msgs.action import DriveDistance, RotateAngle
from math import radians


class BallTracker(Node):
    
    CENTER_TOLERANCE = 30
    ROTATE_SCAN_ANGLE = radians(20)
    MAX_ROTATE_ANGLE = radians(30)

    def __init__(self):
        super().__init__('ball_tracker')

        self.image_width = 640
        self.action=0

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.image_pub = self.create_publisher(Image, '/ball_tracker/image_processed', 10)

        self._action_client_drive = ActionClient(self, DriveDistance, 'my_drive_distance')
        self._action_client_rotate = ActionClient(self, RotateAngle, 'my_rotate_angle')

        self.target_position = None
        self.obstacle_detected = False
        self.processing = False

        self.timer = self.create_timer(0.5, self.main_loop)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            height, width, _ = cv_image.shape
            self.image_width = width

            # Red color mask
            lower_red1 = np.array([0, 150, 120])
            upper_red1 = np.array([5, 255, 255])
            lower_red2 = np.array([175, 150, 120])
            upper_red2 = np.array([180, 255, 255])
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, lower_red1, upper_red1),
                cv2.inRange(hsv, lower_red2, upper_red2)
            )

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest)
                self.target_position = x + w // 2
                self.get_logger().info(f"🎯 Balle rouge détectée à {self.target_position}")
            else:
                self.target_position = None
                self.get_logger().info("❌ Aucune balle rouge détectée")

            # Debug display
            cv2.imshow("Image d'origine", cv_image)
            cv2.imshow("Masque Rouge", mask)
            cv2.waitKey(1)

            processed_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.image_pub.publish(processed_image)

        except Exception as e:
            self.get_logger().error(f"Erreur de traitement d'image: {e}")

    def laser_callback(self, msg):
        self.obstacle_detected = min(msg.ranges) < 0

    def main_loop(self):
        if self.processing:
            return

        if self.target_position is not None:
            center_x = self.image_width // 2
            error = self.target_position - center_x
            self.get_logger().info(f"Erreur de centrage: {error}")

            if abs(error) > self.CENTER_TOLERANCE:
                angle = max(min(-error * 0.005, self.MAX_ROTATE_ANGLE), -self.MAX_ROTATE_ANGLE)
                self.get_logger().info(f"↪️ Correction de l'angle: {angle:.2f} rad")
                self.send_goal_rotate(angle)
            else:
                if not self.obstacle_detected:
                    self.get_logger().info("⬆️ Balle centrée, on avance")
                    self.send_goal_drive(0.2)
                else:
                    self.get_logger().info("🛑 Obstacle détecté, on ne bouge pas")
        else:
            self.get_logger().info("🔍 Balle non visible, balayage")
            self.search_ball()
    def search_ball(self):
            if self.action%4==0:
                self.send_goal_rotate(self.ROTATE_SCAN_ANGLE)
            else:
                self.send_goal_drive(1.0)
            self.action+=1    
            
    

    def send_goal_drive(self, distance: float):
        self.processing = True
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance

        self._action_client_drive.wait_for_server()
        future = self._action_client_drive.send_goal_async(goal_msg, feedback_callback=self.drive_feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def send_goal_rotate(self, angle: float):
        self.processing = True
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle

        self._action_client_rotate.wait_for_server()
        future = self._action_client_rotate.send_goal_async(goal_msg, feedback_callback=self.rotate_feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('❌ Goal rejected')
            self.processing = False
            return

        self.get_logger().info('✅ Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'🎯 Action terminée. Résultat: {result}')
        self.processing = False

    def drive_feedback_callback(self, feedback_msg):
        remaining = feedback_msg.feedback.remaining_travel_distance
        self.get_logger().info(f'🚗 Avance... Reste: {remaining:.2f} m')

    def rotate_feedback_callback(self, feedback_msg):
        remaining = feedback_msg.feedback.remaining_angle_travel
        self.get_logger().info(f'🔄 Rotation... Reste: {remaining:.2f} rad')


def main(args=None):
    rclpy.init(args=args)
    ball_tracker = BallTracker()
    rclpy.spin(ball_tracker)
    ball_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
