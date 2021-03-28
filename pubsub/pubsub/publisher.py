import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self._pub = self.create_publisher(String, 'hello_ros', 10)

        period = 1 # in s
        timer = self.create_timer(period, self.timer_cb)
        

    def timer_cb(self):
        msg = String()
        msg.data = 'Hello! ROS2 is fun'
        self._pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()
    rclpy.spin(publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
