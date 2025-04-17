from irobot_create_msgs.action import DriveDistance, RotateAngle
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse


class MoveController(Node):

    def __init__(self):
        super().__init__('move_controller')

        self._drive_action_server = ActionServer(
            self,
            DriveDistance,
            'my_drive_distance',
            self.execute_drive_distance_callback,
           
        )
        
        self._drive_action_client = ActionClient(
            self,
            DriveDistance,
            'drive_distance'
        )
       
        self._rotate_action_server = ActionServer(
            self,
            RotateAngle,
            'my_rotate_angle',
            self.execute_rotate_angle_callback,
        )


        self._rotate_action_client = ActionClient(
            self,
            RotateAngle,
            'rotate_angle'
        )

       
        self.is_executing = False  
         
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def execute_drive_distance_callback(self, goal_handle):
        """Execute a goal."""

        self.is_executing=True
        self.get_logger().info(f"Executing drive distance: {goal_handle.request.distance}m")

        goal_msg = DriveDistance.Goal()
        goal_msg.distance = goal_handle.request.distance

        self._drive_action_client.wait_for_server()

        def feedback_callback(feedback_msg):
            self.get_logger().info(f'Feedback: {feedback_msg.feedback}')
            goal_handle.publish_feedback(feedback_msg.feedback)

        self._send_goal_future = self._drive_action_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        self.result = DriveDistance.Result()

        def get_result_callback(future):
            self.result = future.result().result
            self.get_logger().info('Drive Completed. Result: {0}'.format(self.result))
            goal_handle.succeed()
            self.is_executing = False    
            self.get_logger().info('Allowing new requests.')

        def goal_response_callback(future):
            goal_handle_response = future.result()
            if not goal_handle_response.accepted:
                self.get_logger().info('Drive goal rejected :(')
                goal_handle.abort()
                self.is_executing = False
                self.get_logger().info('Drive rejected. Allowing new requests.')
                return

            self.get_logger().info('Drive goal accepted.')
            self._get_result_future = goal_handle_response.get_result_async()
            self._get_result_future.add_done_callback(get_result_callback)

        self._send_goal_future.add_done_callback(goal_response_callback)
        
        while self.is_executing:
            pass

        return self.result

    def execute_rotate_angle_callback(self, goal_handle):
        self.is_executing=True
        self.get_logger().info(f"Executing rotate: {goal_handle.request.angle}°")

        goal_msg = RotateAngle.Goal()
        goal_msg.angle = goal_handle.request.angle

        self._rotate_action_client.wait_for_server()

        def feedback_callback(feedback_msg):
            self.get_logger().info(f'Feedback: {feedback_msg.feedback}')
            goal_handle.publish_feedback(feedback_msg.feedback)

        self._send_goal_future = self._rotate_action_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        self.result = RotateAngle.Result()

        def get_result_callback(future):
            self.result = future.result().result
            self.get_logger().info(f'Rotate completed. Result: {self.result}')
            goal_handle.succeed()
            self.is_executing = False
            self.get_logger().info('Allowing new requests. ')

        def goal_response_callback(future):
            goal_handle_response = future.result()
            self.get_logger().info('Goal response received: {0}'.format(goal_handle_response))

            if not goal_handle_response.accepted:
                self.get_logger().info('Rotate goal rejected :(')
                goal_handle.abort()
                self.is_executing = False  # Reset flag
                self.get_logger().info('Rotate rejected. Allowing new requests.')
                return

            self.get_logger().info('Rotate goal accepted.')
            self._get_result_future = goal_handle_response.get_result_async()
            self._get_result_future.add_done_callback(get_result_callback)

        self._send_goal_future.add_done_callback(goal_response_callback)

        while self.is_executing:
            pass

        return self.result


def main(args=None):
    rclpy.init(args=args)
    move_controller = MoveController()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(move_controller)
    executor.spin()

    move_controller.destroy() 
    rclpy.shutdown()


if __name__ == '__main__':
    main()
