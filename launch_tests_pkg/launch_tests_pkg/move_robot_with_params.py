import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
import argparse


class MoveRobot(Node):

    def __init__(self):
        super().__init__('test')

        self.declare_parameter('turning_speed', 0.1)
        self.declare_parameter('forward_speed', 0.1)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd = Twist()

        self.getting_params()

        self.timer_period = 1.0
        self.mode = "turning"

        self.create_timer(self.timer_period, self.timer_callback)

    def getting_params(self):

        self.turning_speed = self.get_parameter(
            'turning_speed').get_parameter_value().double_value
        self.forward_speed = self.get_parameter(
            'forward_speed').get_parameter_value().double_value

        self.get_logger().info("############### turning_speed...."+str(self.turning_speed))
        self.get_logger().info("############### forward_speed...."+str(self.forward_speed))

    def timer_callback(self):
        """
        Change of mode each time this callback is triggered
        """
        if self.mode == "turning":
            self.go_forwards()
            self.mode = "go_forward"
        elif self.mode == "go_forward":
            self.turn()
            self.mode = "turning"
        else:
            pass

    def turn(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = self.turning_speed
        self.get_logger().info("TURNING...."+str(self.cmd.angular.z))
        self.publisher_.publish(self.cmd)

    def go_forwards(self):
        self.cmd.linear.x = self.forward_speed
        self.cmd.angular.z = 0.0
        self.get_logger().info("GOING FORWARDS...."+str(self.cmd.linear.x))
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