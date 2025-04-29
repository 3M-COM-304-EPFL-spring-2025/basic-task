import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
import numpy as np
from ultralytics import YOLO

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')

        self.bridge = CvBridge()

        # YOLO model - detects 80 COCO classes
        self.model = YOLO('yolov8s.pt')  # You can use yolov8s.pt for better accuracy

        # Set up publisher for visualization
        self.marker_pub = self.create_publisher(Marker, 'ball_marker', 10)

        # Subscribers to RGB and Depth
        self.rgb_sub = Subscriber(self, Image, '/oakd/rgb/preview/image_raw')
        self.depth_sub = Subscriber(self, Image, '/oakd/rgb/preview/depth')
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.image_callback)

        # TF buffer to convert camera frame → map
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Ball detector node started.")

    def publish_marker(self, point_map):
        marker = Marker()
        marker.header.frame_id = point_map.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ball_detector"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point_map.point.x
        marker.pose.position.y = point_map.point.y
        marker.pose.position.z = point_map.point.z
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)
        self.get_logger().info("Marker published.")

    def image_callback(self, rgb_msg, depth_msg):
        # Convert to OpenCV
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # Detect ball
        u, v = self.run_yolo(rgb_image)

        if u is None:
            self.get_logger().info("Ball not found.")
            return

        # Get depth value at (u, v)
        try:
            z = depth_image[v, u] / 1000.0  # convert mm to meters
        except IndexError:
            self.get_logger().warn("Index out of bounds in depth image.")
            return

        if z == 0 or np.isnan(z):
            self.get_logger().warn("Invalid depth.")
            return

        # Camera intrinsics (you may need to calibrate these)
        fx, fy = 400.0, 400.0
        cx, cy = 320.0, 240.0

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # Point in camera frame
        point_camera = PointStamped()
        point_camera.header = depth_msg.header
        point_camera.point.x = x
        point_camera.point.y = y
        point_camera.point.z = z

        try:
            point_map = self.tf_buffer.transform(
                point_camera, 'map', timeout=rclpy.duration.Duration(seconds=1.0))
            self.get_logger().info(
                f"Ball position in map: x={point_map.point.x:.2f}, y={point_map.point.y:.2f}")
            
            # Publish visualization marker
            self.publish_marker(point_map)
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {str(e)}")

    def run_yolo(self, image):
        """Run YOLO and return (u, v) pixel of sports ball."""
        results = self.model(image, verbose=False)[0]

        for box in results.boxes:
            class_id = int(box.cls[0])
            if class_id == 32:  # 32 = sports ball in COCO
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                u = (x1 + x2) // 2
                v = (y1 + y2) // 2
                return u, v

        return None, None

def main(args=None):
    print("Starting ball detector node 2...")
    rclpy.init(args=args)
    node = BallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
