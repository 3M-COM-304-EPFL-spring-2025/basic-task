from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
#from geometry_msgs.msg import Vector3
import rclpy
import depthai as dai
import math

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector_node')

        self.ball_detected_pub = self.create_publisher(Bool, '/ball_detected', 10)
        self.ball_position_pub = self.create_publisher(Float32MultiArray, '/ball_position', 10)

        self.pipeline = dai.Pipeline()

        # Color Camera
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(30)
        cam_rgb.setPreviewSize(416, 416)

        # Mono cameras + Stereo depth
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)

        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  #Fix depthai alignment warning
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # YOLO Detection Network
        detection_nn = self.pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        detection_nn.setBlobPath("yolov8n_coco_416x416.blob")
        detection_nn.setConfidenceThreshold(0.5)
        detection_nn.setNumClasses(1)
        detection_nn.setCoordinateSize(4)
        detection_nn.setAnchors([10,14, 23,27, 37,58, 81,82, 135,169, 344,319])
        detection_nn.setAnchorMasks({"side20": [0,1,2], "side10": [3,4,5]})
        detection_nn.setIouThreshold(0.5)
        detection_nn.setBoundingBoxScaleFactor(0.5)
        detection_nn.setDepthLowerThreshold(100)
        detection_nn.setDepthUpperThreshold(5000)

        cam_rgb.preview.link(detection_nn.input)
        stereo.depth.link(detection_nn.inputDepth)

        # Outputs
        xout_nn = self.pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("detections")
        detection_nn.out.link(xout_nn.input)

        self.device = dai.Device(self.pipeline)
        self.q_detections = self.device.getOutputQueue(name="detections", maxSize=3, blocking=False)

        self.timer = self.create_timer(1, self.process)

    def process(self):
        in_detections = self.q_detections.get()
        detected = Bool()
        ball_info = Float32MultiArray()

        detections = in_detections.detections
        for detection in detections:
            if detection.label == 32:  # COCO class for sports ball, frisbee, and orange. YOLO detects our red ball as either of those.
                detected.data = True

                x = detection.spatialCoordinates.x / 1000.0  # in meters
                z = detection.spatialCoordinates.z / 1000.0

                distance = math.sqrt(x**2 + z**2)
                angle_deg = math.degrees(math.atan2(x, z)) + 90

                ball_info.data = [distance, angle_deg]
                break
        else:
            detected.data = False
            ball_info.data = [0.0, 0.0]

        self.ball_detected_pub.publish(detected)
        self.ball_position_pub.publish(ball_info)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
    while True:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            break
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
