# import the SetBool module from std_servs service interface
from pickle import TRUE
from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node
import time
import argparse

class DummyArgumetsExample(Node):

    def __init__(self, args):

        self.timer_flag = True
        
        super().__init__('dummy_arguments_example')

        # More info here: https://docs.python.org/3/library/argparse.html
        parser = argparse.ArgumentParser(
            description='Dummy Example for Arguments use')
        
        # All the arguments in this group will only be allowed to pass one of them as argument
        source = parser.add_mutually_exclusive_group(required=True)
        source.add_argument('-file', type=str, metavar='FILE_NAME',
                            help='Load entity xml from file')
        source.add_argument('-name_server', type=str, metavar='ENTITY_NAME',
                            help='Load entity name')

        # Warning: Setting the Nargs makes the variable now a list
        parser.add_argument('-timer_period_message',
                            type=str,
                            nargs=2,
                            help='Time the service will be waiting',
                            required=True)
        
        # Metvar will replace the default NAME_OF_ARGUMENT with the value shown in the command parser.print_help()

        parser.add_argument('-timer_period', 
                            type=float,
                            metavar='TIMEOUT', 
                            default=1.0,                           
                            help="Time period of the callback for timer")
        
        self.args = parser.parse_args(args[1:])

        # Check length of timer period message is 2
        assert len(self.args.timer_period_message) >= 2 , "You have to place two words"
        parser.print_help()

        self.timer = self.create_timer(float(self.args.timer_period), self.timer_callback)

    def timer_callback(self):
        self.print_dummy_msgs()

    def print_dummy_msgs(self):
        if self.timer_flag:
            self.get_logger().info(self.get_name()+"---"+self.args.timer_period_message[0])
            self.timer_flag = False
        else:
            self.get_logger().info(self.get_name()+"---"+self.args.timer_period_message[1])
            self.timer_flag = True

    def get_name(self):

        # Check which one was used and fill the var:
        final_name = "Nothing"
        try:            
            final_name = self.args.file
        except:
            pass

        try:            
            final_name = self.args.name_server
        except:
            pass  

        return final_name


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    print("args==="+str(args))
    # format the arguments given through ROS to use the arguments
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    print("clean ROS args==="+str(args_without_ros))
    dummy_args_node = DummyArgumetsExample(args_without_ros)
    dummy_args_node.get_logger().info(" Started")
    # parser the program execution and wait for a request to kill the node (ctrl+c)
    rclpy.spin(dummy_args_node)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()