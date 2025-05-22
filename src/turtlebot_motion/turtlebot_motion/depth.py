from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
import rclpy
import depthai as dai
import numpy as np

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector_node')

        self.ball_detected_pub = self.create_publisher(Bool, '/ball_detected', 10)
        self.ball_position_pub = self.create_publisher(Vector3, '/ball_position', 10)

        self.pipeline = dai.Pipeline()

        # Color Camera
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(30)

        # Mono cameras + Stereo depth
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)

        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # YOLO Detection Network
        detection_nn = self.pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        detection_nn.setBlobPath("/home/mikail/yolov8n_coco_416x416.blob")  # Set this to your compiled blob
        detection_nn.setConfidenceThreshold(0.5)
        detection_nn.setNumClasses(1)
        detection_nn.setCoordinateSize(4)
        detection_nn.setAnchors([10,14, 23,27, 37,58, 81,82, 135,169, 344,319])
        detection_nn.setAnchorMasks({"side20": [0,1,2], "side10": [3,4,5]})
        detection_nn.setIouThreshold(0.5)
        detection_nn.input.setBlocking(False)
        detection_nn.setBoundingBoxScaleFactor(0.5)
        detection_nn.setDepthLowerThreshold(100)
        detection_nn.setDepthUpperThreshold(5000)

        cam_rgb.preview.link(detection_nn.input)

        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        xout_nn = self.pipeline.create(dai.node.XLinkOut)

        xout_rgb.setStreamName("rgb")
        xout_depth.setStreamName("depth")
        xout_nn.setStreamName("detections")

        cam_rgb.preview.link(xout_rgb.input)
        stereo.depth.link(xout_depth.input)
        detection_nn.out.link(xout_nn.input)

        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.q_depth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        self.q_detections = self.device.getOutputQueue(name="detections", maxSize=4, blocking=False)

        self.timer = self.create_timer(0.1, self.process)

    def process(self):
        in_rgb = self.q_rgb.get()
        in_depth = self.q_depth.get()
        in_detections = self.q_detections.get()

        frame = in_rgb.getCvFrame()
        depth_frame = in_depth.getFrame()

        detected = Bool()
        position = Vector3()

        detections = in_detections.detections
        if detections:
            detection = detections[0]
            x1 = int(detection.xmin * frame.shape[1])
            y1 = int(detection.ymin * frame.shape[0])
            x2 = int(detection.xmax * frame.shape[1])
            y2 = int(detection.ymax * frame.shape[0])

            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cx = np.clip(cx, 0, depth_frame.shape[1] - 1)
            cy = np.clip(cy, 0, depth_frame.shape[0] - 1)

            distance_mm = depth_frame[cy, cx]
            position.z = distance_mm / 1000.0

            image_center_x = frame.shape[1] // 2
            fov = 68.8
            position.x = ((cx - image_center_x) / image_center_x) * (fov / 2)
            position.y = 0.0

            detected.data = True
        else:
            detected.data = False
            position.x = position.y = position.z = 0.0

        self.ball_detected_pub.publish(detected)
        self.ball_position_pub.publish(position)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
