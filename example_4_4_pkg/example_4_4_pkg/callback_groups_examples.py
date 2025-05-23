# import the SetBool module from std_servs service interface
from pickle import TRUE
from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node
import time

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile

import argparse

# Basic Docs:
# https://docs.ros2.org/foxy/api/rclpy/api/execution_and_callbacks.html

class DummyServer(Node):

    def __init__(self, args, callback_group_type="reentrant"):

        self.timer_flag = True

        

        super().__init__('service_start_turn')

        # More info here: https://docs.python.org/3/library/argparse.html
        parser = argparse.ArgumentParser(
            description='Dummy Server to Learn about Callback Groups and Threads')
        
        # All the arguments in this group will only be allowed to pass one of them as an argument

        parser.add_argument('-service_wait_time',
                            type=float, 
                            #default=5.0,
                            help='Time the service will be waiting',
                            required=True)
        
        # Metvar will replace the default NAME_OF_ARGUMENT with the value shown in the command parser.print_help()
        # Warning: Setting the Nargs makes the variable now a list
        parser.add_argument('-timer_period', 
                            type=float, 
                            nargs=1,
                            metavar='TIMEOUT', 
                            default=1.0,                           
                            help="Time period of the Callback for the timer")
        
        parser.add_argument('-callback_group_type', 
                            type=str, 
                            default="reentrant",                           
                            help="Type of Callback Groups REENTRANT of EXCLUSIVE")
        
        parser.add_argument('-threads', 
                            type=int, 
                            default=1,                           
                            help="Number of threads to use in the executor")
        
        self.args = parser.parse_args(args[1:])

        parser.print_help()

        # <rclpy.callback_groups.MutuallyExclusiveCallbackGroup object at 0x7ff58fc9e8e0>
        # By default, the Callbacks are mutually exclusive. This means that in each group, only
        # one Callback can be done: https://docs.ros2.org/foxy/api/rclpy/api/node.html
        print("## DEFAULT Node Callback Group="+str(self.default_callback_group))

        self.get_logger().warning("Setting "+self.args.callback_group_type+" Groups")
        if self.args.callback_group_type == "reentrant":       
            # If you set the group reentrant, any Callback inside will be executed in parallel
            # If there are enough threads     
            self.group1 = ReentrantCallbackGroup()
            self.get_logger().warning("ReentrantCallbackGroup Set")
            self.srv = self.create_service(SetBool, '/dummy_server_srv', self.SetBool_callback, callback_group=self.group1)
            self.timer = self.create_timer(self.args.timer_period[0], self.timer_callback, callback_group=self.group1)

        elif self.args.callback_group_type == "exclusive":            
            self.group1 = MutuallyExclusiveCallbackGroup()
            self.group2 = MutuallyExclusiveCallbackGroup()
            self.get_logger().warning("MutuallyExclusiveCallbackGroup Set")
            self.srv = self.create_service(SetBool, '/dummy_server_srv', self.SetBool_callback, callback_group=self.group1)
            self.timer = self.create_timer(self.args.timer_period[0], self.timer_callback, callback_group=self.group2)

        else:
            # You do not set groups. Therefore, they will get the default group for the Node
            self.get_logger().error("NO GROUPS SET Set")
            self.srv = self.create_service(SetBool, '/dummy_server_srv', self.SetBool_callback)
            self.timer = self.create_timer(self.args.timer_period[0], self.timer_callback)



    def get_threads(self):
        return self.args.threads
    
        

    def SetBool_callback(self, request, response):
        self.get_logger().warning("Processing Server Message...")
        self.wait_for_sec(self.args.service_wait_time)
        self.get_logger().warning("Processing Server Message...DONE")
        response.message = 'TURNING'
        # return the response parameters
        return response
    
    def wait_for_sec(self, wait_sec, delta=1.0):
        i = 0
        while i < wait_sec :
             self.get_logger().info("..."+str(i)+"[WAITING...]")
             time.sleep(delta)
             i += delta

    def timer_callback(self):
        self.print_dummy_msgs()

    def print_dummy_msgs(self):
        if self.timer_flag:
            self.get_logger().info("TICK")
            self.timer_flag = False
        else:
            self.get_logger().info("TACK")
            self.timer_flag = True


def mutualexclusive_demo_fun(args=None):
    # To Use: ros2 service call /dummy_server_srv std_srvs/srv/SetBool data:\ false\
    # ros2 run execution_and_callbacks_examples mutualexclusive_demo_fun -service_wait_time 5.0 -timer_period 1.0 
    # initialize the ROS communication
    rclpy.init(args=args)
    print("args==="+str(args))
    # Format the arguments given through ROS to use the arguments
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    print("clean ROS args==="+str(args_without_ros))
    start_stop_service_node = DummyServer(args_without_ros)

    num_threads = start_stop_service_node.get_threads()
    start_stop_service_node.get_logger().info('DummyServer Started with threads='+str(num_threads))

    executor = MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(start_stop_service_node)
        
    try:
        executor.spin()
    finally:
        executor.shutdown()
        start_stop_service_node.destroy_node()
            
    
    # shutdown the ROS communication
    rclpy.shutdown()

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()