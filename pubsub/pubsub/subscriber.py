import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.create_subscription(String, 'hello_ros', self.sub_cb, 10)
      
    def sub_cb(self, msg):
        self.get_logger().info('Msg Received : ' +  msg.data)


def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()
    rclpy.spin(subscriber)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
