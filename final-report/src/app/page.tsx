import React from "react";
import ReportCard from "./components/ReportCard";
import SectionHeader from "./components/SectionHeader";
import ListItem from "./components/ListItem";
import Figure from "./components/Figure";
import CodeBlock from "./components/CodeBlock";
import VideoPlayer from "./components/VideoPlayer";

import img_robotBall from "../../public/robot-ball.png";
import img_waypointsMap from "../../public/waypoints-map.png";
import img_depthai from "../../public/depthai.png";
import img_rvizAmphi from "../../public/rviz-amphi.png";
import img_360Spin from "../../public/360-spin.png";
import img_rvizCost from "../../public/rviz-cost.png";

const AutonomousReport: React.FC = () => {
  return (
    <div className="text-gray-900 px-6 md:px-16 py-12 max-w-7xl mx-auto space-y-12">
      {/* Header Section */}
      <header className="text-center">
        <h1 className="text-4xl font-extrabold text-gray-900 mb-2">
          Autonomous Target Navigation with Object Delivery to a Human
        </h1>
        <p className="text-md text-gray-600">
          Maryam Harakat (359826), Mohamed Taha Guelzim (355739), Muhammad
          Mikail Rais (402800)
        </p>
        <p className="text-sm text-gray-500 mt-1">
          COM-304 Final Project Report
        </p>
      </header>

      {/* Project Video */}
      <div className="flex flex-row justify-around h-96 items-center mb-8">
        <ReportCard title="Project Demonstration Video 1">
          <VideoPlayer
            src={
              "https://www.dropbox.com/scl/fi/uehd7sd0t8rih7a6fepmf/demo.mp4?rlkey=9ie2myxne7icui8tqcht8mly6&st=ws9td4c1&dl=0"
            }
          />
        </ReportCard>
        <ReportCard title="Project Demonstration Video 2">
          <VideoPlayer
            src={
              "https://www.dropbox.com/scl/fi/aadkmvjqnbxyelrbg7mut/demo2.mp4?rlkey=mhopp9hw6pp8j92qvx1gv1og0&st=taikmze2&dl=0"
            }
          />
        </ReportCard>
      </div>

      {/* Introduction Image */}
      <Figure
        src={img_robotBall}
        alt="Robot stopped in front of the target ball."
        caption="Robot stopped in front of the target ball."
        width={800}
        height={450}
      />

      {/* Abstract */}
      <ReportCard title="Abstract">
        <p className="text-justify">
          This project implements a real-time ball detection and tracking system
          for a mobile robot using the OAK-D Lite stereo camera and the ROS 2
          Humble middleware. The systemleverages on-device YOLO inference
          provided by DepthAI, stereo depth data, and odometry feedback to
          estimate the relative position of a ball in 2D space. The main output
          of the system is a boolean flag indicating detection, and a position
          vector consisting of the estimated angle and distance to the ball.
          This data can be used for downstream navigation or manipulation tasks.
        </p>
      </ReportCard>

      {/* Table of Contents */}
      <ReportCard title="Table of Contents">
        <ol className="list-decimal list-inside space-y-2">
          <ListItem>
            <a href="#introduction" className="text-blue-500 hover:underline">
              Introduction
            </a>
          </ListItem>
          <ListItem>
            <a href="#related-work" className="text-blue-500 hover:underline">
              Related Work
            </a>
          </ListItem>
          <ListItem>
            <a href="#method" className="text-blue-500 hover:underline">
              Method
            </a>
          </ListItem>
          <ListItem>
            <a href="#experiments" className="text-blue-500 hover:underline">
              Experiments
            </a>
          </ListItem>
          <ListItem>
            <a href="#limitations" className="text-blue-500 hover:underline">
              Limitations
            </a>
          </ListItem>
          <ListItem>
            <a
              href="#individual-contributions"
              className="text-blue-500 hover:underline"
            >
              Individual Contributions
            </a>
          </ListItem>
          <ListItem>
            <a href="#extension" className="text-blue-500 hover:underline">
              Extension
            </a>
          </ListItem>
          <ListItem>
            <a href="#conclusion" className="text-blue-500 hover:underline">
              Conclusion
            </a>
          </ListItem>
        </ol>
      </ReportCard>

      {/* Introduction Section */}
      <SectionHeader title="Introduction" id="introduction" />
      <ReportCard title="A. What is the problem we want to solve?">
        <p className="text-justify">
          This project aims to develop an autonomous robot capable of navigating
          through obstacle-filled environments to locate, push, and deliver a
          predefined object to a moving human recipient. Unlike traditional
          navigation tasks, this requires dynamic target tracking and human
          interaction. The key challenge is designing a solution that doesn’t
          require specialized hardware, such as robotic arms or custom pushing
          mechanisms. This raises important research questions:
        </p>
        <ul className="list-disc pl-6">
          <li>How does a robot perceive its environment?</li>
          <li>
            How can a robot effectively navigate through and manipulate objects
            in mentioned environment without specialized hardware?
          </li>
          <li>
            How can it track and follow a moving human recipient in real-time
            while ensuring accurate delivery?
          </li>
        </ul>
      </ReportCard>
      <ReportCard title="B. Why is it important that this problem be solved?">
        <p className="text-justify">
          Solving this problem is vital for advancing autonomous robots in
          human-centric applications, such as assisting elderly or disabled
          individuals, automating package delivery, and streamlining warehouse
          logistics. By eliminating the need for hardware modifications, our
          solution enables easy deployment on existing platforms, making it
          adaptable and scalable. This research addresses the broader question:
        </p>
        <ul className="list-disc pl-6">
          <li>
            How can we make autonomous robots more versatile and efficient
            without requiring hardware changes, ensuring seamless integration
            into real-world environments?
          </li>
        </ul>
      </ReportCard>
      <ReportCard title="C. Problem Reiteration and Primary Objectives">
        <p>
          Ball detection and tracking is a fundamental problem in mobile
          robotics, particularly in applications involving object following,
          pick-and-place, or navigation toward targets. This project integrates
          real-time object detection, depth perception, and motion compensation
          to track a red sports ball using the OAK-D Lite camera and a TurtleBot
          4 Lite running ROS 2 Humble. Hence, the primary objectives in this
          projects were:
        </p>
        <ul className="list-disc pl-6">
          <li>Detect a ball using YOLO on the OAK-D Lite VPU.</li>
          <li>Estimate the ball’s 2D relative position using stereo depth.</li>
          <li>Integrate odometry to compensate for processing delay.</li>
          <li>Publish clean outputs suitable for robot navigation logic.</li>
        </ul>
      </ReportCard>

      {/* Related Work Section */}
      <SectionHeader title="Related Work" id="related-work" />
      <ReportCard title="A. Luxonis DepthAI Framework">
        <p className="text-justify">
          DepthAI is an embedded AI vision platform enabling stereo depth and
          neural inference on-device via the Myriad X processor. Our system uses
          DepthAI’s YoloSpatialDetectionNetwork to perform real-time object
          detection and depth estimation on-device.
        </p>
      </ReportCard>
      <ReportCard title="B. Visual Servoing and Object Following with ROS">
        <p className="text-justify">
          The objective of this work—detect and track objects in real time using
          a mobile robot—is similar to our project goal, although their work
          relies on traditional vision methods (color segmentation, optical
          flow) and CPU-based processing. We improved robustness and speed by
          using depth sensing and YOLO, leading to more accurate and
          lower-latency tracking.
        </p>
      </ReportCard>
      <ReportCard title="C. VSLAM and Odometry Integration">
        <p className="text-justify">
          Mur-Artal et al (2015) presented their solution, ORB- SLAM, which uses
          visual data along with odometry to build accurate spatial
          understanding over time.
        </p>
      </ReportCard>

      {/* Method Section */}
      <SectionHeader title="Method" id="method" />
      <ReportCard title="A. Space Exploration and Mapping">
        <p className="text-justify">
          To enable autonomous space exploration, our system integrates
          Simultaneous Localization and Mapping (SLAM) with custom frontier
          detection and navigation strategies. SLAM is a core technique that
          allows the robot to construct a map of an unknown environment while
          concurrently estimating its own position within that map. In our
          implementation, we utilized a SLAM algorithm in conjunction with ROS2
          and RViz, enabling the construction and real-time visualization of a
          2D occupancy grid. This grid map is generated using LiDAR data and
          categorizes the environment into free space, obstacles, and unknown
          regions. The exploration process begins with the initial mapping
          phase, where SLAM builds a foundational representation of the
          environment. A custom algorithm is then applied to detect
          frontiers—boundaries between known and unexplored areas of the map.
        </p>
        <CodeBlock
          code={`
@staticmethod
def convolute(data, coverage_map, coordinates, size=3, occ_threshold=40, coverage_weight=0.5):
    """
    Performs a convolution operation on the occupancy grid data to determine if a waypoint is accessible.
    """
    occ_sum = 0
    coverage_sum = 0

    for x in range(int(coordinates[0] - size / 2), int(coordinates[0] + size / 2)):
        for y in range(int(coordinates[1] - size / 2), int(coordinates[1] + size / 2)):
            if data[x, y] == -1:
                occ_sum += 100  # unknown area
            elif data[x, y] > 50:
                occ_sum += 1000000  # obstacle
            else:
                occ_sum += data[x, y]

            # encourage going to unseen places (0 = unseen, 1 = seen)
            coverage_sum += 1 - coverage_map[x, y]  # high if unseen

    area = size * size
    occ_avg = occ_sum / area
    coverage_avg = coverage_sum / area  # 0 if fully seen, 1 if fully unseen

    if occ_avg < occ_threshold:
        score = (1 - coverage_weight) * occ_avg + coverage_weight * (100 * coverage_avg)
        return True, score
    else:
        return False, float('inf')
            `}
          title="discoverer.py - convolute function"
          githubUrl="https://github.com/3M-COM-304-EPFL-spring-2025/basic-task/blob/7fb19cfdb68b19b6690625ff41fc7c46f32ad7f3/code/discoverer.py#L391"
        />

        <p className="text-justify">
          Once identified, frontiers are evaluated based on specific selection
          criteria. The size of each frontier is assessed, with preference given
          to larger frontiers that potentially open access to broader unexplored
          areas. After a frontier is selected, the robot navigates to it using
          the Nav2 navigation stack. Navigation is supported by a costmap, which
          incorporates obstacle data to ensure safe and collision-free movement.
        </p>

        <CodeBlock
          code={`
goal_msg = NavigateToPose.Goal()
goal_msg.pose.header.frame_id = 'odom'
goal_msg.pose.pose.position.x = float(
    waypoint[0] + self.cartographer.origin[0])
goal_msg.pose.pose.position.y = float(
    waypoint[1] + self.cartographer.origin[1])
          `}
          title="discoverer.py - goal_msg construction"
          githubUrl="https://github.com/3M-COM-304-EPFL-spring-2025/basic-task/blob/7fb19cfdb68b19b6690625ff41fc7c46f32ad7f3/code/discoverer.py#L584"
        />

        <p className="text-justify">
          To help debugging and enhance situational awareness, we created a
          secondary map that visualizes waypoints and distinguishes between
          accessible and non-accessible points. This secondary map provides
          critical insight into the robot’s decision-making process during
          exploration and navigation.
        </p>

        <Figure
          src={img_waypointsMap}
          alt="Waypoints Map"
          caption="Secondary map showing waypoints (dots), accessibility (red/green), current map (gray), frontiers (yellow) and target (blue star)."
          width={800}
          height={450}
        />
      </ReportCard>
      <ReportCard title="B. Hardware-Native Object Detection">
        <p className="text-justify">
          A notable achievement in our project is that we managed to run YOLO
          inference natively on the Turtlebot camera. This enabled the robot to
          do real-time object detection while simultaneously run SLAM to map the
          environment. Firstly, we installed DepthAI library on the Turtlebot’s
          Raspberry Pi. DepthAI is an open-source framework developed by Luxonis
          that provides a way to interface with DepthAI hardware, one of which
          is the OakD Lite camera on Turtlebot. Because DepthAI devices use
          Intel’s Myriad X VPU, it only runs models compiled into a special
          binary format called .blob which stands for binary large object, but
          in the DepthAI world, it is a compiled model ready to run on-device.
          It is generated from standard model formats like ONNX, TensorFlow, or
          OpenVINO IR using a tool called blobconverter. Luxonis has an open
          model zoo that contains various object detection models, among them
          are YOLOv8n and YOLOv7. We had compared the performance and
          reliability of both model and decided to use YOLOv8n.
        </p>
        <Figure
          src={img_depthai}
          alt="DepthAI Software showing Yolo model running on the camera, along with stereo depth estimation."
          caption="DepthAI Software showing Yolo model running on the camera, along with stereo depth estimation."
          width={800}
          height={450}
        />
        <p className="text-justify">
          The DepthAI framework provides a convenient Python API to interact
          with the camera and run inference. We used the
          YoloSpatialDetectionNetwork class to create a pipeline that captures
          images, runs YOLO inference, and outputs detection results with
          spatial coordinates. The pipeline also provides depth information for
          each detected object, allowing us to estimate the 2D position of the
          ball in the robot’s coordinate frame.
        </p>
        <CodeBlock
          code={`
class BallDetector(Node):
def __init__(self):
    super().__init__('ball_detector_node')
    self.ball_detected_pub = self.create_publisher(Bool, '/ball_detected', 10)
    self.ball_position_pub = self.create_publisher(Float32MultiArray, '/ball_position', 10)

    self.pipeline = dai.Pipeline()
    cam_rgb = self.pipeline.create(dai.node.ColorCamera)
    mono_left = self.pipeline.create(dai.node.MonoCamera)
    mono_right = self.pipeline.create(dai.node.MonoCamera)
    stereo = self.pipeline.create(dai.node.StereoDepth)
    detection_nn = self.pipeline.create(dai.node.YoloSpatialDetectionNetwork)

    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    cam_rgb.preview.link(detection_nn.input)
    stereo.depth.link(detection_nn.inputDepth)

    xout_nn = self.pipeline.create(dai.node.XLinkOut)
    xout_nn.setStreamName("detections")
    detection_nn.out.link(xout_nn.input)

    self.device = dai.Device(self.pipeline)
    self.q_detections = self.device.getOutputQueue(name="detections")

    self.create_timer(1, self.process)

def process(self):
    if not self.q_detections.has():
        return

    detections = self.q_detections.get().detections
    detected = Bool()
    ball_pos = Float32MultiArray()

    for d in detections:
        if d.label == 39:
            detected.data = True
[...]
`}
          title="simplified depth.py - BallDetector class"
          githubUrl="https://github.com/3M-COM-304-EPFL-spring-2025/basic-task/blob/7fb19cfdb68b19b6690625ff41fc7c46f32ad7f3/code/depth.py#L13"
        />

        <p className="text-justify">
          The ball detection module runs continuously, capturing images and
          processing them in real-time.
        </p>
      </ReportCard>
      <ReportCard title="C. Navigation to Detected Ball">
        <p className="text-justify">
          <span className="font-bold">1. Locating Ball Target: </span>
          During navigation, the robot captures images at regular intervals as
          it moves through the environment. Once the robot reaches a selected
          frontier point (as given by the Nav2 navigation stack feedback), it
          performs a 360-degree rotation to further enhance its ability to
          detect any balls in its surroundings. This rotation ensures that the
          robot scans the entire environment, maximizing the likelihood of
          spotting a ball from any angle.
        </p>
        <br />
        <p className="text-justify">
          <span className="font-bold">2. Acquiring Ball Target: </span>
          Upon detecting the ball, we infer its position in the map. In the
          previous code snippet, we saw how we could get the angle of the ball’s
          center relative to the camera frame and the distance from the robot to
          the ball. Additionally, odometry correction is applied to account for
          the robot’s movement and ensure accurate positioning.
        </p>
        <CodeBlock
          code={`*
        x = d.spatialCoordinates.x / 1000
        z = d.spatialCoordinates.z / 1000
        dist = (x**2 + z**2) ** 0.5
        angle = math.atan2(x, z)

        if hasattr(self, 'latest_odom'):
            p = self.latest_odom.pose.pose.position
            q = self.latest_odom.pose.pose.orientation
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            ball_x = p.x + dist * math.cos(yaw + angle)
            ball_y = p.y + dist * math.sin(yaw + angle)
        else:
            ball_x = ball_y = 0.0

        ball_pos.data = [ball_x, ball_y]
        break
else:
    detected.data = False
    ball_pos.data = [0.0, 0.0]

self.ball_detected_pub.publish(detected)
self.ball_position_pub.publish(ball_pos)
          `}
          title="simplified depth.py - Ball position calculation"
          githubUrl="https://github.com/3M-COM-304-EPFL-spring-2025/basic-task/blob/7fb19cfdb68b19b6690625ff41fc7c46f32ad7f3/code/depth.py#L98"
        />
        <p className="text-justify">
          Once the ball’s coordinates are established, the robot uses the Nav2
          navigation stack again to plot a direct path to the ball’s location.
          This ensures that the robot efficiently moves toward the ball,
          adjusting its trajectory as necessary based on its real-time
          localization and the environment.
        </p>
      </ReportCard>

      {/* Experiment Section */}
      <SectionHeader title="Experiments" id="experiments" />
      <ReportCard title="Component Testing">
        <p className="text-justify">
          As explained above, our project is composed of three main components:
          autonomous exploration of the environment, visual recognition of a
          colored ball, and navigation toward the detected ball. Each of these
          components addresses a fundamental robotics task, and each brings its
          own technical challenges. To develop a robust and reliable system, we
          adopted a modular and hypothesis-driven approach: we first implemented
          and tested each module independently, under controlled conditions, and
          then integrated them for full pipeline testing in diverse real-world
          environments. We deliberately chose to test each component in
          isolation before integration for two main reasons. First, this allowed
          us to identify and debug issues specific to each sub-system without
          the confounding influence of the others. Second, we hypothesized that
          performance bottlenecks and failure cases would be easier to interpret
          when isolated, making debugging and improvements more efficient. This
          hypothesis was largely confirmed in practice: by testing components
          separately, we were able to refine frontier exploration strategies,
          improve visual recognition robustness, and tune navigation parameters
          without cascading failures.
        </p>
      </ReportCard>
      <ReportCard title="Exploration Module">
        <p className="text-justify">
          ‍The goal of the exploration module was to enable the robot to
          autonomously build a map of an unknown environment using SLAM and then
          use frontier-based planning to explore the entire space. We
          hypothesized that frontier-based exploration would generalize well
          across a variety of environments with different spatial structures. To
          test this, we deployed the robot in a wide range of real-world
          settings, including large open areas (such as the basement of the CO
          building), small rooms, classrooms, amphitheaters, and living spaces.
          We evaluated the quality of the exploration by visualizing the
          SLAM-generated maps in RViz and comparing them to the actual layout of
          the environment.
        </p>
        <Figure
          src={img_rvizAmphi}
          alt="RViz visualization of the SLAM map in an amphitheater."
          caption="RViz visualization of the SLAM map in an amphitheater."
          width={400}
          height={200}
        />
        <p className="text-justify">
          Our criteria included completeness of the exploration, the relevance
          of frontier points, and the consistency of the map structure. In early
          trials, the robot explored small and medium-sized rooms reliably, but
          failed in large spaces like the CO building basement—remaining in a
          small area and re-scanning already visited zones. This refuted our
          initial hypothesis that the exploration algorithm would generalize to
          all indoor environments. Debugging revealed the issue: the waypoint
          list wasn’t updated correctly as the map expanded, causing the robot
          to revisit outdated frontiers. We fixed this by (1) dynamically
          maintaining the waypoint list to remove stale entries and add new ones
          as the map grew, and (2) scaling the waypoint map to cover the full
          known environment.
        </p>
        <CodeBlock
          code={`
            # reshape the data so it resembles the map shape
    data = np.reshape(data, (current_map_height, current_map_width))
    self.waypoints = self.generate_waypoints_from_map(
        current_map_width, current_map_height, resolution)
    waypoints_height = max(self.waypoints[:, 0])
    waypoints_width = max(self.waypoints[:, 1])
    self.get_logger().info(
        f"Waypoints height: {waypoints_height}, Waypoints width: {waypoints_width}")
        `}
          title="discoverer.py - Waypoint list maintenance"
          githubUrl="https://github.com/3M-COM-304-EPFL-spring-2025/basic-task/blob/7fb19cfdb68b19b6690625ff41fc7c46f32ad7f3/code/discoverer.py#L287C9-L294C89"
        />
        <p className="text-justify">
          After these changes, exploration succeeded even in large, complex
          areas, validating our original hypothesis—once the underlying bug was
          resolved.
        </p>
      </ReportCard>
      <ReportCard title="Ball Detection Module">
        <p className="text-justify">
          For the recognition module, we chose to detect the ball using YOLO as
          stated before. We hypothesized that this method would be sufficient
          for real-time detection in indoor settings, and that lighting
          variations or partial occlusions would be the primary challenges. We
          first tested ball detection with the robot’s camera detached and
          stationary. We placed the ball in various positions: in plain sight,
          partially hidden, far from the camera, and close to cluttered
          objects.These initial tests confirmed that our recognition pipeline
          worked reliably under controlled lighting. However, when we reattached
          the camera to the moving robot and re-ran the same tests, we
          encountered new challenges. Motion blur and angle variations
          introduced noise into the detection pipeline. In the first
          implementation, during the 360° image capture, there wasn’t enough
          time of stabilization, photographs were therefore taken while the
          camera was partially in motion, and the resulting frames exhibited
          pronounced blur. The practical consequence was a drop in
          ball-detection recall during early scans. We hypothesized that the
          blur was caused not by sensor noise or lighting but by insufficient
          settling time before each shot. To test this, we inserted a short
          “settle 1 s-and-shoot” pause: the robot halts for 1 second, leaving
          time for frame capture, and only then advances to the next yaw
          increment. Subsequent experiments confirmed the hypothesis. Like this,
          we could correctly compute the relative coordinates of the ball when
          detected.
        </p>
        <Figure
          src={img_360Spin}
          alt="Robot performing a 360° rotation to capture 6 images."
          caption="Robot performing a 360° rotation to capture 6 images."
          width={500}
          height={450}
        />
      </ReportCard>
      <ReportCard title="Navigation Module">
        <p className="text-justify">
          The navigation component is responsible for planning and executing a
          path from the robot’s current position to the detected ball. Our
          hypothesis here was that once the ball is successfully localized in
          space, the existing path-planning algorithm (based on the robot’s SLAM
          map and obstacle avoidance) would be sufficient to approach the target
          reliably. To test this, we placed the ball in various positions within
          the robot’s field of view and initiated the navigation routine. The
          ball was placed in open areas, near walls, partially occluded, and
          among obstacles (but visible to the robot). The navigation system
          performed relatively well when the path to the ball was relatively
          clear and the SLAM map accurately reflected the environment. However,
          in cluttered settings or when the ball was placed very close to
          obstacles, the robot occasionally failed to compute a viable path or
          hesitated in its approach. By default, the cost map inflates
          obstacles—that is, it adds a safety buffer around them to prevent
          collisions.
        </p>
        <Figure
          src={img_rvizCost}
          alt="RViz visualization of the cost map with inflated obstacles."
          caption="RViz visualization of the cost map with inflated obstacles."
          width={800}
          height={450}
        />
        <p className="text-justify">
          However, in our case, the inflation radius was too large: objects
          appeared significantly bigger than they actually were, making narrow
          passages or cluttered environments effectively impassable in the
          planner’s view. As a result, when the ball was placed near obstacles,
          the robot often struggled to find a viable path, or took unnecessarily
          long detours. In practice, this made navigation toward the ball less
          reliable in constrained spaces. Fortunately, we identified this issue
          and have been able to reduce the cost and tweak with additionnal
          paramters from the NavPy config file.
        </p>
        <CodeBlock
          code={`
        <param name="inflation_radius" type="double" value="0.2"/>
<param name="obstacle_range" type="double" value="3.0"/>
        `}
          title="nav2_params.yaml - Costmap parameters"
        />
      </ReportCard>
      <ReportCard title="Integration Testing">
        <p className="text-justify">
          After thoroughly validating each component on its own, we moved on to
          integrated testing of the complete pipeline. These end-to-end tests
          were performed in a variety of real-world environments, such as
          classrooms, amphitheaters, and corridors in the CO building. The aim
          was to assess the overall coherence of the system—specifically,
          whether the transition between exploration, detection, and navigation
          occurred smoothly and whether the modules could function together
          without interference. These integrated experiments also allowed us to
          observe cumulative system behavior, such as total execution time,
          responsiveness, and robustness under varying spatial and visual
          conditions.
        </p>
      </ReportCard>

      {/* Conclusion Section */}
      <SectionHeader title="Limitations" id="limitations" />
      <ReportCard title="Hardware Limitations">
        <p className="text-justify">
          A significant limitation of our project was persistent hardware and
          connectivity problems. We lost valuable development time due to issues
          such as missing wheels, malfunctioning LiDAR and camera sensors, and
          frequent failures in maintaining a stable connection with the robot.
          These setbacks delayed progress and prevented us from dedicating more
          time to advanced functionalities we had initially planned. If the
          hardware had functioned correctly from the outset, we could have
          implemented more robust features and carried out more extensive
          testing.
        </p>
      </ReportCard>
      <ReportCard title="Inaccurate Ball Detection">
        <p className="text-justify">
          Our depth calculations were slightly inaccurate when images were
          captured while the robot was rotating. This was primarily due to
          motion blur and synchronization issues with odometry data. The robot's
          movement introduced inconsistencies between the captured image and the
          odometry feedback, leading to errors in the computed depth and
          position of the ball. To mitigate this, we should have averaged the
          computed positions from the last x images instead of relying solely on
          the first computed position. By aggregating data over multiple frames,
          we could have reduced noise and improved the reliability of the ball's
          estimated coordinates.
        </p>
      </ReportCard>
      <ReportCard title="Visual Coverage Mapping">
        <p className="text-justify">
          While the mapping algorithm performed well in terms of obstacle
          avoidance and constructing accurate maps, we did not observe
          substantial improvements in exploration behavior due to visual
          coverage mapping.
        </p>

        <CodeBlock
          code={`
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
        `}
          title="discoverer.py - Visual coverage mapping"
          githubUrl="https://github.com/3M-COM-304-EPFL-spring-2025/basic-task/blob/bcfc9eef212eb5b70766f456cdf410e34beaa17c/code/visual_coverage_mapper.py#L14"
        />

        <Figure
          src={img_rvizAmphi}
          alt="Visual coverage map in gray showing areas seen by the camera."
          caption="Visual coverage map in gray showing areas seen by the camera."
          width={300}
          height={450}
        />

        <p className="text-justify">
          It seemed as though the algorithm was not fully leveraging the visual
          coverage data to optimize its exploration strategy. However, during
          our testing, the robot's 360-degree rotation behavior proved
          sufficient to cover the majority of the environment. This rotation
          allowed the robot to scan its surroundings effectively, ensuring that
          most areas were explored without requiring additional adjustments to
          the exploration algorithm.
        </p>
      </ReportCard>

      {/* Individual Contributions */}
      <SectionHeader
        title="Individual Contributions"
        id="individual-contributions"
      />
      <ReportCard title="Maryam">
        <p>
          Maryam was primarily responsible for designing and implementing the
          core exploration algorithm. She developed the initial logic allowing
          the robot to autonomously explore unknown environments and identify
          frontier regions for navigation. She also implemented the 360° camera
          sweep mechanism at exploration start-up, enabling the robot to rotate
          on the spot and capture panoramic visual data. Furthermore, she
          integrated the logic to periodically capture images every 3 meters
          during exploration, laying the foundation for coupling vision with
          mapping.
        </p>
      </ReportCard>

      <ReportCard title="Mohamed Taha">
        <p>
          Mohamed Taha played a major role in debugging critical issues,
          including waypoint list maintenance, ensuring dynamic frontier updates
          as the SLAM map expanded, and resolving hardware connection problems
          between the robot and the controlling software stack. His
          contributions were essential to enabling robust and scalable
          exploration. He also worked on the development of the visual coverage
          map, a feature intended to track which areas had been seen by the
          camera, as distinct from those simply scanned by LiDAR.
        </p>
      </ReportCard>

      <ReportCard title="Mikail">
        <p>
          Mikail was in charge of the ball recognition pipeline using a
          YOLO-based object detection model. He handled the integration of the
          visual detector with the robot’s camera feed, and implemented the
          logic to extract 2D image coordinates of the ball, which could later
          be projected into real-world navigation targets. His work was crucial
          to enabling the robot to visually detect and locate its goal object.
        </p>
      </ReportCard>

      <ReportCard title="Maryam and Mohamed Taha">
        <p>
          Maryam and Mohamed Taha jointly conducted extensive testing across
          diverse real-world environments, including classrooms, amphitheaters,
          hallways, and basements. Their tests helped validate the system’s
          robustness and uncover limitations under varying spatial and visual
          conditions.
        </p>
      </ReportCard>

      {/*Extension Section */}
      <SectionHeader title="Extension" id="extension" />
      <ReportCard title="Delivering Objects">
        <p className="text-justify">
          While we were unable to implement the extension of delivering objects
          to a human recipient due to time and hardware constraints, we
          identified a practical and simplified approach to achieve this
          functionality within the limits of our robot's capabilities. The
          delivery process would be based on straight-path planning and simple
          object pushing, with continuous feedback correction.
        </p>
        <ol className="list-decimal list-inside space-y-2">
          <li>
            <span className="font-bold">Fixed Human Position: </span>
            To mitigate the challenge of potential occlusion caused by the
            object during pushing, the recipient’s position would be assumed to
            be fixed. This avoids the need for continuous visual tracking, which
            may not be possible once the object blocks the camera's field of
            view.
          </li>
          <li>
            <span className="font-bold">Straight Path Planning: </span>
            The robot would compute the shortest, straight-line path from its
            current location to the fixed human position. This simplifies path
            planning and avoids the need for complex dynamic updates.
          </li>
          <li>
            <span className="font-bold">Straight Pushing Behavior: </span>
            Object delivery would rely on pushing the object directly along the
            planned path. During this phase, the robot would only move forward
            in a straight line while maintaining pressure against the object.
          </li>
          <li>
            <span className="font-bold">LIDAR-Based Recalibration: </span>
            The robot would use LIDAR data to monitor the alignment of the
            object during pushing. If the object begins to tilt to one side, the
            robot would slightly rotate in the opposite direction to re-center
            the object, ensuring more stable and accurate pushing.
          </li>
          <li>
            <span className="font-bold">Continuous Path Correction: </span>
            Although the target path is initially computed as a straight line,
            minor adjustments would be made throughout the pushing process to
            correct for drift and maintain overall alignment with the target
            direction.
          </li>
        </ol>
        <p className="text-justify">
          This approach offers a lightweight and achievable path toward object
          delivery with limited hardware. Future work could refine the
          LIDAR-based correction strategy, explore partial human tracking
          pre-push, and evaluate delivery reliability under varied terrain and
          object shapes.
        </p>
      </ReportCard>

      {/* Conclusion Section */}
      <SectionHeader title="Conclusion" id="conclusion" />
      <ReportCard title="A. Summary of Findings">
        <p className="text-justify">
          In this project, we successfully developed a mobile robot capable of
          autonomously navigating through complex environments to locate a given
          target object. The system integrates real-time ball detection, depth
          estimation, and odometry correction to achieve accurate localization
          and navigation. Our findings indicate that the combination of
          YOLO-based object detection and stereo depth sensing provides a robust
          solution for real-time target tracking, enabling the robot to
          effectively navigate towards the detected ball.
        </p>
      </ReportCard>
      <ReportCard title="B. Future Work">
        <p className="text-justify">
          Future work should focus on enhancing the robustness of the navigation
          system, particularly in cluttered environments. This could involve
          refining the cost map parameters, improving the ball detection
          algorithm to handle occlusions and lighting variations, and
          integrating more advanced path-planning techniques. Additionally,
          exploring alternative hardware configurations and sensors could
          further improve the system’s performance and adaptability.
        </p>
      </ReportCard>
      <ReportCard title="C. Final Thoughts">
        <p className="text-justify">
          Overall, this project demonstrates the potential of combining
          hardware-native object detection with real-time depth sensing and
          odometry correction to enable autonomous navigation and manipulation
          tasks in robotics. The successful integration of these components
          highlights the importance of modular design and hypothesis-driven
          testing in developing robust robotic systems.
        </p>
      </ReportCard>

      {/* Footer */}
      <footer className="text-center text-sm text-gray-500 mt-12">
        3M - COM304 Project • 2025
      </footer>
    </div>
  );
};

export default AutonomousReport;
