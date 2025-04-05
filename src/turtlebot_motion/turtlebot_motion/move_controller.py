from irobot_create_msgs.action import DriveDistance, RotateAngle, Undock
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse
from threading import Lock

class MoveController(Node):

    def __init__(self):
        super().__init__('move_controller')

        # Track ongoing executions
        self.is_executing = False  
        self.execution_lock = Lock()  

        # Action server for undocking
        self._undock_action_server = ActionServer(
            self,
            Undock,
            'my_undock',
            self.undock,
            goal_callback=self.goal_callback
        )

        # Action client for undocking
        self._undock_action_client = ActionClient(
            self,
            Undock,
            'undock'
        )

        # Action server for driving distance
        self._drive_action_server = ActionServer(
            self,
            DriveDistance,
            'my_drive_distance',
            self.execute_drive_distance_callback,
            goal_callback=self.goal_callback
        )
        
        # Action client for driving distance
        self._drive_action_client = ActionClient(
            self,
            DriveDistance,
            'drive_distance'
        )
        
        # Action server for rotating angle
        self._rotate_action_server = ActionServer(
            self,
            RotateAngle,
            'my_rotate_angle',
            self.execute_rotate_angle_callback,
            goal_callback=self.goal_callback
        )

        self._rotate_action_client = ActionClient(
            self,
            RotateAngle,
            'rotate_angle'
        )

    def undock(self, goal_handle):
        self.get_logger().info('Undocking the robot...')
        self._undock_action_client.wait_for_server()
        
        undock_goal = Undock.Goal()
        self._goal_future = self._undock_action_client.send_goal_async(undock_goal)
        self.result = Undock.Result()

        def undock_result_callback(future):
            self.result.is_docked = future.result().result.is_docked
            
            if not self.result.is_docked:
                self.get_logger().info('Undock completed successfully.')
                goal_handle.succeed()
            
            else:
                self.get_logger().warn('Undock failed. Please undock manually if necessary.')
                goal_handle.abort()   

            with self.execution_lock:
                self.is_executing = False
                self.get_logger().info('Allowing new requests.')           
                
        def undock_response_callback(future):
            goal_handle_response = future.result()
            if not goal_handle_response.accepted:
                self.get_logger().warn('Undock goal rejected, please undock manually if necessary.')
                goal_handle.abort()

                with self.execution_lock:
                    self.is_executing = False
                    self.get_logger().info('Undock rejected. Allowing new requests.')
                return

            self.get_logger().info('Undock goal accepted.')
            self._get_result_future = goal_handle_response.get_result_async()
            self._get_result_future.add_done_callback(undock_result_callback)

        self._goal_future.add_done_callback(undock_response_callback)

        while self.is_executing:
            pass

        return self.result
        

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        with self.execution_lock:
            if self.is_executing:
                self.get_logger().info("New drive distance request rejected: Another action is in progress.")
                return GoalResponse.REJECT

            self.is_executing = True
        return GoalResponse.ACCEPT

    def execute_drive_distance_callback(self, goal_handle):
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

            with self.execution_lock:
                self.is_executing = False  # Allow new requests
                self.get_logger().info('Allowing new requests.')

        def goal_response_callback(future):
            goal_handle_response = future.result()
            if not goal_handle_response.accepted:
                self.get_logger().info('Drive goal rejected :(')
                goal_handle.abort()

                with self.execution_lock:
                    self.is_executing = False  # Reset flag
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

            with self.execution_lock:
                self.is_executing = False
                self.get_logger().info('Allowing new requests. ')

        def goal_response_callback(future):
            goal_handle_response = future.result()
            self.get_logger().info('Goal response received: {0}'.format(goal_handle_response))
            if not goal_handle_response.accepted:
                self.get_logger().info('Rotate goal rejected :(')
                goal_handle.abort()

                with self.execution_lock:
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

    move_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
