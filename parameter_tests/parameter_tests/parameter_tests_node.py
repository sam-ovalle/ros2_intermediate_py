import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class VelParam(rclpy.node.Node):

    def __init__(self):
        super().__init__('param_vel_node')
        self.timer = self.create_timer(2, self.timer_callback)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.msg = Twist()
        param_descriptor = ParameterDescriptor(
            description='Sets the velocity (in m/s) of the robot.')
        self.declare_parameter('velocity', 0.0, param_descriptor)
        self.add_on_set_parameters_callback(self.parameter_callback)

    def timer_callback(self):
        self.my_param = self.get_parameter('velocity').value

        self.get_logger().info('Velocity parameter is: %f' % self.my_param)

        self.msg.linear.x = self.my_param
        self.publisher.publish(self.msg)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'velocity' and param.type_ == Parameter.Type.DOUBLE:
                self.my_param = param.value
                self.get_logger().info('Velocity parameter changed!')
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = VelParam()
    rclpy.spin(node)


if __name__ == '__main__':
    main()