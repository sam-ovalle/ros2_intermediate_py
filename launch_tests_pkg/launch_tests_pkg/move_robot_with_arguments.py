import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
import argparse


class MoveRobot(Node):

    def __init__(self, args):
        super().__init__('test')

        self.argument_parsing(args)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd = Twist()

        self.timer_period = 1.0
        self.mode = "turning"

        self.create_timer(self.timer_period, self.timer_callback)

    def argument_parsing(self, args):
        parser = argparse.ArgumentParser(
            description='Dummy Example for Arguments use')

        parser.add_argument('-turning_speed',
                            type=float,
                            metavar='1.0',
                            default=1.0,
                            help="The Turning speed in radians per second")
        parser.add_argument('-forward_speed',
                            type=float,
                            metavar='1.0',
                            default=1.0,
                            help="The Move Forward speed in meters per second")

        self.args = parser.parse_args(args[1:])

    def timer_callback(self):
        """
        Change the mode each time this callback is triggered.
        """
        if self.mode == "turning":
            self.go_forwards()
            self.mode = "go_forward"
        if self.mode == "go_forward":
            self.turn()
            self.mode = "turning"
        else:
            pass

    def turn(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = self.args.turning_speed
        self.get_logger().info("TURNING...."+str(self.cmd.angular.z))
        self.publisher_.publish(self.cmd)

    def go_forwards(self):
        self.cmd.linear.x = self.args.forward_speed
        self.cmd.angular.z = 0.0
        self.get_logger().info("GOING FORWARD...."+str(self.cmd.linear.x))
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
    print("args==="+str(args))

    args_without_ros = rclpy.utilities.remove_ros_args(args)
    print("clean ROS args==="+str(args_without_ros))

    try:
        move_robot_node = MoveRobot(args_without_ros)
        rclpy.spin(move_robot_node)
        move_robot_node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()