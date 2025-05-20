import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveRobot(Node):

    def __init__(self):
        super().__init__('test')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd = Twist()
        self.turn()

    def turn(self):
        self.get_logger().info("TURNING....")
        self.cmd.linear.x = 0.5
        self.cmd.angular.z = 0.5
        self.publisher_.publish(self.cmd)

    def stop(self):
        self.get_logger().info("STOPPING....")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.get_logger().info("STOPPED")

    def __del__(self):
        self.stop()


def main(args=None):
    rclpy.init(args=args)

    try:
        move_robot_node = MoveRobot()
        rclpy.spin(move_robot_node)
        move_robot_node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()