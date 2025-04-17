import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from math import pi


from irobot_create_msgs.action import DriveDistance, RotateAngle

class ComputationController(Node):
    def __init__(self):
        super().__init__('computation_controller')

        self._action_client_drive=ActionClient(self, DriveDistance, 'my_drive_distance')
        self._action_client_rotate=ActionClient(self, RotateAngle, 'my_rotate_angle')

        self.step=0

       


    def send_goal_rotate(self, angle: float):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        feedback_callback = self.rotate_feedback_callback
        _action_client = self._action_client_rotate
        _action_client.wait_for_server()
        send_goal_future = _action_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        

    def send_goal_drive(self, distance:float):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance
        feedback_callback = self.drive_feedback_callback
        _action_client = self._action_client_drive
        _action_client.wait_for_server()
        send_goal_future = _action_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        """ Handles goal response. """
        goal_handle = future.result()
        self.get_logger().info('Goal accepted!')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """ Handles action completion. """
        result = future.result().result
        self.get_logger().info(f'Action completed! Result: {result}')

        self.move_triangle()
        self.step+=1
        


    def drive_feedback_callback(self, feedback_msg):
        """ Handles feedback for drive action. """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Driving... Remaining: {feedback.remaining_travel_distance}m')

    def rotate_feedback_callback(self, feedback_msg):
        """ Handles feedback for rotate action. """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Rotating... Remaining: {feedback.remaining_angle_travel}°')



    def move_triangle(self):
        if self.step<18:
            if self.step % 6==4 or self.step%6==5:
                self.send_goal_drive(0.25)
                
            else:
                self.send_goal_rotate(pi/6)        
        else: 
            #self.send_goal_drive(0.25) 
            rclpy.shutdown()
            


        

def main(args=None):
    rclpy.init(args=args)

    computation_controller = ComputationController()

    computation_controller.move_triangle()

    rclpy.spin(computation_controller)

if __name__ == '__main__':
    main()
