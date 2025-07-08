import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import Int16, String
from std_srvs.srv import Trigger


class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        # Publisher
        self.pub = self.create_publisher(String, 'cmd', 10)
        
        # Callback Group
        topic_cb_group = ReentrantCallbackGroup()
        srv_cb_group = topic_cb_group
        # 以下2行だと無理だった
        #topic_cb_group = MutuallyExclusiveCallbackGroup()
        #srv_cb_group = MutuallyExclusiveCallbackGroup()
        # Mutuallyに含まれるコールバック同士は並列実行してはいけない
        # Reentrantに含まれるコールバック同士は並列に実行可能

        # Subscriber
        self.create_subscription(Int16, 'num', self.callback, 10, callback_group=topic_cb_group)
        
        # Service
        self.srv = self.create_service(Trigger, 'test_server', self.handle_request, callback_group=srv_cb_group)
        self.get_logger().info('Service <test_server> is ready.')
        
        # Value
        self.cmd = String()
        self.num = None

    def callback(self, msg):
        self.num = msg.data

    def handle_request(self, req, res):
        self.get_logger().info('Executing handle_request')
        
        self.cmd.data = 'pub'
        self.pub.publish(self.cmd)

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
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
