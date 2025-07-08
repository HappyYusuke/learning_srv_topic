import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, String
from std_srvs.srv import Trigger


class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        # Publisher
        self.pub = self.create_publisher(String, 'cmd', 10)
        
        # Subscriber
        self.create_subscription(Int16, 'num', self.callback, 10)
        
        # Service
        self.srv = self.create_service(Trigger, 'test_server', self.handle_request)
        self.get_logger().info('Service <test_server> is ready.')
        
        # Value
        self.cmd = String()
        self.num = None

    def callback(self, msg):
        self.num = msg.num

    def handle_request(self, req, res):
        self.get_logger().info('Executing handle_request')
        
        self.cmd.data = 'pub'
        self.pub.publish(self.cmd)

        # ここで、デッドロックする
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.num == 666:
                break

        res.success = True
        res.message = 'finished'
        self.get_logger().info('finished handle_request')


def main():
    rclpy.init()
    node = ServiceServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
