# written by Enrique Fernández-Laguilhoat Sánchez-Biezma and Daniel García López

#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# 3rd partys
import numpy as np
import pandas as pd
import os
import csv
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from geometry_msgs.msg import Twist
import time
import math


# ros
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image

from rclpy.action import ActionServer, CancelResponse
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterType
from action_msgs.msg import GoalStatus

# messages
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav2_msgs.action import NavigateToPose

from matplotlib import pyplot as plt


# ros2 action send_goal wander explorer_interfaces/action/Wander "{strategy: 1, map_completed_thres: 0.6}"

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



class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cartographer = CartographerSubscriber()  # a cartographer subscription is created to access the occupancy
        rclpy.spin_once(self.cartographer)
        # grid and determine which positions to navigate to

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Exploration goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Arrived at destination')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        rclpy.spin_once(self.cartographer)

    def send_goal(self):


        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        rclpy.spin_once(self.cartographer)  # refresh the list of accessible waypoints
        waypoint = self.cartographer.sorted_accessible_waypoints[0]  # grab the first waypoint
        self.cartographer.sorted_accessible_waypoints = self.cartographer.sorted_accessible_waypoints[1:]  # pop the
        # first element from the list, in case the
        # accessible waypoints didn't refresh
        # write command
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.pose.position.x = float(waypoint[0] + self.cartographer.origin[0])
        goal_msg.pose.pose.position.y = float(waypoint[1] + self.cartographer.origin[1])
        # goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            'Sending navigation goal request x: ' + str(round(goal_msg.pose.pose.position.x, 2)) + ' y: ' + str(
                round(goal_msg.pose.pose.position.y, 2)))

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        rclpy.spin_until_future_complete(self, self._send_goal_future)

        goal_handle = self._send_goal_future.result()
        get_result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, get_result_future)
        return False


class CartographerSubscriber(Node):
    def __init__(self):
        super().__init__('cartographer_subscriber')
        self.occupancy_subscription = self.create_subscription(OccupancyGrid, 'map', self.occupancy_callback, 10)

        self.waypoints = self.generate_list_of_waypoints(n_of_waypoints=100, step=0.2)
        self.accessible_waypoints = np.array([])
        self.sorted_accessible_waypoints = np.array([])
        self.occupancy_value = np.array([])
        self.origin = np.array([0.0, 0.0])

    def occupancy_callback(self, msg):
        """

        The cartographer subscriber callback function refreshes the list of accessible waypoints. It sorts them and
        saves them in the self.sorted_accessible_waypoints variable.

        :param msg: OccupancyGrid message. Includes map metadata and an array with the occupancy probability values
        :return: None
        """

        data = np.array(msg.data)  # download the occupancy grid
        current_map_width = msg.info.width  # get the current map width
        current_map_height = msg.info.height  # get the current map height
        resolution = msg.info.resolution  # get the resolution
        origin = msg.info.origin.position  # get the origin of the map
        self.origin = np.array([origin.x, origin.y])  # save the origin in a numpy array
        # get the map origin in the occupancy grid
        origin_x = int((origin.x) / resolution)
        origin_y = int((origin.y) / resolution)


        # reshape the data so it resembles the map shape
        data = np.reshape(data, (current_map_height, current_map_width))

        # Here we go through every waypoint and save the ones that are accessible.
        # An accessible waypoint is one which has no obstacles, and has few or no unknown squares in the vicinity.
        self.accessible_waypoints = np.array([])
        self.occupancy_value = np.array([])
        unaccessible_waypoints = np.array([])
        for waypoint in self.waypoints:
            try:
                occupancy_grid_coordinates = [int((waypoint[1]) / resolution), int((waypoint[0]) /
                                                                                         resolution)]
                conv = self.convolute(data, occupancy_grid_coordinates, size=9)  # perform convolution

                # if the convolution returns True, it means the WP is accessible, so it is stored in
                # self.accessible_waypoints
                if conv[0]:
                    self.accessible_waypoints = np.append(self.accessible_waypoints, waypoint)
                    self.occupancy_value = np.append(self.occupancy_value, conv[1])
                else:
                    # if the convolution returns False, it means the WP is not accessible, so it is stored in
                    # self.unaccessible_waypoints
                    unaccessible_waypoints = np.append(unaccessible_waypoints, waypoint)
            # because the waypoint array is over-sized, we need to remove the values that are out of range

            except IndexError:
                pass

            # scatter the accessible and unaccessible waypoints in the map with different colors
            """
            plt.scatter(self.accessible_waypoints[:, 0], self.accessible_waypoints[:, 1], c='green', s=1)
            plt.scatter(unaccessible_waypoints[:, 0], unaccessible_waypoints[:, 1], c='red', s=1)
            plt.xlim(0, 2.3)
            plt.ylim(0, 2.3)
            plt.title('Accessible waypoints')
            plt.xlabel('X axis')
            plt.ylabel('Y axis')
            plt.grid()
            plt.pause(0.01)  # Pause to update the plot
            plt.clf()  # Clear the plot for the next iteration
            plt.show()  # Show the plot
            """

        # reshape the accessible waypoints array to shape (n, 2)
        self.accessible_waypoints = self.accessible_waypoints.reshape((-1, 2))

        # Sorting...
        occupancy_value_idxs = self.occupancy_value.argsort()
        self.sorted_accessible_waypoints = self.accessible_waypoints[occupancy_value_idxs[::-1]]

        # Default fallback waypoints
        if np.size(self.sorted_accessible_waypoints) == 0:
            self.sorted_accessible_waypoints = np.array([[1.5, 0.0], [0.0, 1.5], [-1.5, 0.0], [0.0, -1.5]])

        self.get_logger().info('Accessible waypoints have been updated...')

        # --- ⬇️ Visualization Part Starts Here ⬇️ ---
        plt.figure(figsize=(8, 8))
        # Draw the occupancy map
        data_img = np.copy(data)
        data_img[data_img == -1] = 128  # unknown -> gray
        plt.imshow(data_img, cmap='gray', origin='lower')  # Map

        # Draw waypoints
        if self.accessible_waypoints.shape[0] > 0:
            plt.scatter(
                (self.accessible_waypoints[:, 0]) / msg.info.resolution,
                (self.accessible_waypoints[:, 1]) / msg.info.resolution,
                c='green', s=5, label='Accessible'
            )

        if unaccessible_waypoints.shape[0] > 0:
            unaccessible_waypoints = unaccessible_waypoints.reshape(-1, 2)
            plt.scatter(
                (unaccessible_waypoints[:, 0]) / msg.info.resolution,
                (unaccessible_waypoints[:, 1]) / msg.info.resolution,
                c='red', s=5, label='Unaccessible'
            )

        # Draw navigation goal (first in sorted accessible list)
        if self.sorted_accessible_waypoints.shape[0] > 0:
            nav_goal = self.sorted_accessible_waypoints[0]
            plt.scatter(
                [(nav_goal[0]) / msg.info.resolution],
                [(nav_goal[1]) / msg.info.resolution],
                marker='*', c='blue', s=100, label='Navigation Goal'
            )

        # Draw origin
        plt.scatter(
            [self.origin[0] / msg.info.resolution],
            [self.origin[1] / msg.info.resolution],
            marker='o', c='yellow', s=100, label='Origin'
        )

        plt.legend()
        plt.title('Map with Accessible Waypoints and Navigation Goal')
        plt.xlabel('Map X (cells)')
        plt.ylabel('Map Y (cells)')
        plt.grid(False)
        plt.tight_layout()
        plt.pause(0.001)
        plt.clf()

    @staticmethod
    def convolute(data, coordinates, size=3, threshold=40):
        """
        This function calculates the average occupancy probability at 'coordinates' for an area of size (size x size)
        around said point.

        :param data: Occupancy Grid Data (shaped to (x, y) map dimensions)
        :param coordinates: the coordinates of the OccupancyGrid to convolute around
        :param size: size of the kernel
        :param threshold: threshold of accessibility
        :return: True or False, depending on whether the waypoint is accessible or not.
        :return: average: average occupancy probability of the convolution
        """
        sum = 0
        for x in range(int( coordinates[0] - size / 2), int(coordinates[0] + size / 2)):
            for y in range(int(coordinates[1] - size / 2), int(coordinates[1] + size / 2)):
                # if the area is unknown, we add 100 to sum.
                if data[x, y] == -1:
                    sum += 100
                # if occupancy state is above 50 (occupied), we add 1M to the sum so that the robot DOES NOT
                # access areas near walls.
                elif data[x, y] > 50:
                    sum += 1000000
                # if the occupancy state is below 50 and known, just add the value to sum.
                else:
                    sum += data[x, y]

        # average value for the square is computed
        average = sum / (size * size)
        if average < threshold:
            # if the average of the squares is below the threshold, the waypoint is accessible
            return True, average
        else:
            # if the average is above the threshold, the waypoint has either too many unknowns, or an obstacle
            return False, average

    def generate_list_of_waypoints(self, n_of_waypoints, step):
        """

        Generates a grid of waypoints of size ('n_of_waypoints' * 'n_of_waypoints') and step size 'step'

        :param n_of_waypoints: number of total waypoints to generate per side
        :param step: float resolution of the waypoints
        :return waypoints: 2D numpy array of a list of coordinates of size dim x 2,
        where dim is the number of waypoints
        """

        waypoints = np.zeros((n_of_waypoints * n_of_waypoints, 2))

        i = 0
        for index_y in range(n_of_waypoints):
            for index_x in range(n_of_waypoints):
                waypoints[i] = [float(index_x) / (1/step), float(index_y) / (1/step)]
                i += 1

        self.get_logger().info("Grid of waypoints has been generated.")

        return waypoints
    
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
def reset_commands(command: Twist) -> Twist:
    """Resets all Twist commands to zero."""
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0
    return command 

def spin_detect_ball(subscriber : LaserSubscriber, publisher: CmdVelPublisher, command:Twist, camera_subscriber:CameraSubscriber, model:YOLO):
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
    rclpy.init(args=args)

    navigation=NavigationClient()
    laser_subscriber = LaserSubscriber()
    camera_subscriber = CameraSubscriber()
    cmd_vel_publisher = CmdVelPublisher()
    command = Twist()

    while rclpy.ok():
        navigation.send_goal()
        #if spin_detect_ball(laser_subscriber, cmd_vel_publisher, command, camera_subscriber, YOLO("yolov8x.pt")):
        #navigation.get_logger().info("Ball detected, stopping navigation.")
        #break

    navigation.destroy_node()
    laser_subscriber.destroy_node()
    camera_subscriber.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()