import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserSub(Node):
    def __init__(self):
        super().__init__('laser_sub')
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            10
        )

    def callback(self, msg):
        print(f"Received {len(msg.ranges)} scan points.")
        print(f"Right distance: {msg.ranges[0]} meters")
        print(f"Forward distance: {msg.ranges[len(msg.ranges)//4]} meters")
        print(f"Left distance: {msg.ranges[len(msg.ranges)//2]} meters")
        print(f"Back distance: {msg.ranges[3*len(msg.ranges)//4]} meters")
        print("--------------------------------------------------")


rclpy.init()
node = LaserSub()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
