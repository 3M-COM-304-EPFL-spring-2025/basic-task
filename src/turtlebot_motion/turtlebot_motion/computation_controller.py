import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from math import pi
import random

from irobot_create_msgs.action import DriveDistance, RotateAngle, Undock

class ComputationController(Node):
    def __init__(self):
        super().__init__('computation_controller')

        self.move_count = 0

        self._action_clients = {
            'drive': ActionClient(self, DriveDistance, 'my_drive_distance'),
            'rotate': ActionClient(self, RotateAngle, 'my_rotate_angle'),
            'undock': ActionClient(self, Undock, 'my_undock')
        }

        self._actions = {
            'drive': lambda: self.send_goal('drive', 0.25),
            'left': lambda: self.send_goal('rotate', pi / 6),
            'right': lambda: self.send_goal('rotate', -pi / 6)
        }

    def send_goal(self, goal_type: str, value: float = 0.0):
        """ Sends a goal to either DriveDistance or RotateAngle. """

        if goal_type == 'drive':
            goal_msg = DriveDistance.Goal()
            goal_msg.distance = value
            feedback_callback = self.drive_feedback_callback

        elif goal_type == 'rotate':
            goal_msg = RotateAngle.Goal()
            goal_msg.angle = value
            feedback_callback = self.rotate_feedback_callback

        elif goal_type == 'undock':
            goal_msg = Undock.Goal()
            feedback_callback = None

        else:
            self.get_logger().warn('Invalid goal type!')
            return

        _action_client = self._action_clients[goal_type]

        if not _action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f'{goal_type.capitalize()} action server is not available!')
            return

        self.get_logger().info(f'Sending {goal_type} goal: {value}')

        send_goal_future = _action_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """ Handles goal response. """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected! Will shutdown...')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted!')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """ Handles action completion. """
        result = future.result().result
        self.get_logger().info(f'Action completed! Result: {result}')

        self.send_random_action()

    def drive_feedback_callback(self, feedback_msg):
        """ Handles feedback for drive action. """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Driving... Remaining: {feedback.remaining_travel_distance}m')

    def rotate_feedback_callback(self, feedback_msg):
        """ Handles feedback for rotate action. """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Rotating... Remaining: {feedback.remaining_angle_travel}°')

        """
        def send_random_action(self):
        """ 
        " Randomly selects an action to send."
        " First actions sent are defined, to move the robot away from the origin."
        """
        self.move_count += 1

        if self.move_count == 1:
            # Undocking the robot automatically caused issues on my end (probably due to my simulation low RTS). It may work for you. Try if desired.
            self.get_logger().info('Please refer to the code comments (line 97) or the README.md file for more information about automatic undocking.')
            if False : # Set to True to enable undocking
                self.get_logger().info('Undocking the robot...')
                self.send_goal('undock')
                return
            else:
                self.get_logger().info('Skipping automatic undocking...')

        if self.move_count < 4:
            self._actions['left']()
            self.get_logger().info(f'Sending left rotation ({self.move_count}/3)')

        elif self.move_count < 7:
            self._actions['drive']()
            self.get_logger().info(f'Sending drive action ({self.move_count}/3) to move away from origin...')

        else :
            action = random.choice(list(self._actions.keys()))
            self._actions[action]()
            self.get_logger().info(f'Sending {self.move_count}-th random action: {action}')"""
    
    def send_random_action(self):
        """"
        "Use keyboard to control the robot."
        """

        command = input("Enter command (d/l/r,u): ")
        while command not in ['d', 'l', 'r', 'u']:
            command = input("Invalid command! Enter d, l, or r: ")
        self.move_count += 1
        if command == 'd':
            self._actions['drive']()
            self.get_logger().info('Sending drive action')

        elif command == 'l':
            self._actions['left']()
            self.get_logger().info('Sending left rotation')

        elif command == 'r':
            self._actions['right']()
            self.get_logger().info('Sending right rotation')
        elif command == 'u':
            self.get_logger().info('Undocking the robot...')
            self.send_goal('undock')
            return
        else:
            self.get_logger().warn('Invalid command! Will shutdown...')


def main(args=None):
    rclpy.init(args=args)

    computation_controller = ComputationController()

    computation_controller.send_random_action()

    rclpy.spin(computation_controller)

if __name__ == '__main__':
    main()
