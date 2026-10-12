import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscription(Node):
    def __init__(self):
        super().__init__('minimal_subscription')
        self.subscription = self.create_subscription(String,'chatter',self.listener_callback,10)
    def listener_callback(self,msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscription = MinimalSubscription()
    rclpy.spin(minimal_subscription)
    minimal_subscription.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    