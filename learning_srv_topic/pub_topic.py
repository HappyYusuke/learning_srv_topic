import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, String

class TopicPublisher(Node):
    def __init__(self):
        super().__init__('topic_publisher_node')
        # Publisher
        self.pub = self.create_publisher(Int16, 'num', 10)
        # Subscriber
        self.create_subscription(String, 'cmd', self.callback, 10)
        # Value
        self.msg = Int16()

    def callback(self, msg):
        cmd = msg.data

        if cmd == 'pub':
            num = 666
            self.msg.data = num
            self.get_logger().info(f'Publishing >>> {num}')
            for _ in range(10):
                if not rclpy.ok:
                    break
                self.pub.publish(self.msg)
                time.sleep(0.1)
            self.get_logger().info(f'Published >>> {num}')
            res = True
        else:
            self.get_logger().warn(f'cmd <{cmd}> is not define.')
            res = False


def main():
    rclpy.init()
    node = TopicPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
