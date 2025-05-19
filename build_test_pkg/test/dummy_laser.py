import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import random


class DummyLaser(Node):

    def __init__(self):
        super().__init__('DummyLaser')
        self.count = 0

        self.publisher = self.create_publisher(LaserScan, '/dummy_scan', 10)
        self.timer = self.create_timer(1.0, self.callback)

    def callback(self):
        data = random.random()
        data2 = random.random()
        msg = LaserScan()
        msg.ranges = [data, data2]
        self.get_logger().info('Publishing: '+str((msg.ranges)))
        # We comment this and there fore noone will be publishing in the **/dummy_scan topic**
        # self.publisher.publish(msg)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    node = DummyLaser()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()