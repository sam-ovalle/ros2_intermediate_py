# import the SetBool module from std_servs service interface
from std_srvs.srv import SetBool
# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the ROS2 python client libraries
import rclpy
from rclpy.node import Node
import time
import random
import math

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class MoveServiceServer(Node):

    def __init__(self, cmd_vel_topic="/cmd_vel", timer_period = 0.5):

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()


        # Here we have the class constructor
        self._cmd_vel_topic = cmd_vel_topic
        # call the class constructor to initialize the node as service_moving
        super().__init__('service_start_turn')
        # create the service server object
        # defines the type, name and callback function
        self.srv = self.create_service(SetBool, '/start_turn', self.SetBool_callback, callback_group=self.group1)
       
        # create the publisher object
        # in this case the publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        # use the Twist module
        self.publisher_ = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        # create a Twist message
        self.cmd = Twist()

        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.group2)
    
        

    def SetBool_callback(self, request, response):
        self.get_logger().warning("Processing Service Server Message...")
        # Publish the message to the topic
        # As you see the name of the request parameter is data so let's do it
        if request.data == True:
            # It turns
            self.cmd.linear.x = 0.0
            self.cmd.angular.z =-0.3
            
            
            # We need a response
            response.success = True
            # We need another response but this time SetBool let us put an String
            response.message = 'TURNING'

        if request.data == False:

            self.cmd.linear.x = 0.0
            self.cmd.angular.z =0.3
            
            response.success = True
            response.message = 'GOING FORWARD'

        self.wait_for_sec(15.0)
        self.get_logger().warning("Processing Service Server Message...DONE")
        # We reset to stop once finished
        self.cmd.linear.x = 0.0
        self.cmd.angular.z =0.0
        # return the response parameters
        return response
    
    def wait_for_sec(self, wait_sec, delta=1.0):
        i = 0
        while i < wait_sec :
             self.get_logger().info("..."+str(i)+"[MoveServiceServer]")
             time.sleep(delta)
             i += delta

    def timer_callback(self):
        self.publisher_.publish(self.cmd)
        # Display the message on the console
        self.print_cmd()

    def print_cmd(self):
        self.get_logger().info("[ X="+str(self.cmd.linear.x)+", A="+str(self.cmd.angular.z)+" ]")


class CalculationsServiceServer(Node):

    def __init__(self, cmd_vel_topic="/cmd_vel"):

        self.group3 = MutuallyExclusiveCallbackGroup()
        self.group4 = MutuallyExclusiveCallbackGroup()
        # Here we have the class constructor
        self._cmd_vel_topic = cmd_vel_topic
        self.cmd = Twist()
        # call the class constructor to initialize the node as service_moving
        super().__init__('service_calculations')
        # create the service server object
        # defines the type, name and callback function
        self.srv = self.create_service(SetBool, '/calculations', self.Calculate_callback, callback_group=self.group3)

        self.publisher_ = self.create_publisher(Twist, self._cmd_vel_topic, 10)

        # create the subscriber object
        self.laser_forward = 0
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.get_lase_front, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.group4)


    def get_lase_front(self,msg):
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[359]
        #self.get_logger().warning("Laser Forwards NOW Calback="+str(self.laser_forward))
        #self.get_logger().warning(str(self.laser_forward))


    def Calculate_callback(self, request, response):
        self.get_logger().warning("Processing CalculationsServiceServer Message...")
        old_dist = self.laser_forward
        self.wait_for_sec(5.0)
        new_dist = self.laser_forward
        self.get_logger().warning("Processing CalculationsServiceServer Message...DONE..")

        self.get_logger().warning("Front Laser Value 5 second in the past="+str(old_dist))
        self.get_logger().warning("Front Laser Value now="+str(new_dist))

        delta_dist = new_dist - old_dist
        
        result_msg = ""

        if delta_dist > 0.0:
            result_msg = "Move forwards! delta > 0 "
            self.get_logger().warning(result_msg)
            self.cmd.linear.x = 0.2
            self.cmd.angular.z =0.0
        else:
            result_msg = "Move back! delta <= 0 "
            self.get_logger().warning(result_msg)
            self.cmd.linear.x = -0.2
            self.cmd.angular.z =0.0
        
        # We need another response but this time SetBool let us put an String
        response.message = result_msg

        self.publisher_.publish(self.cmd)
        # return the response parameters
        return response
    
    def wait_for_sec(self, wait_sec, delta=1.0):
        i = 0
        while i < wait_sec :
             self.get_logger().info("..."+str(i)+"[CalculationsServiceServer]")
             time.sleep(delta)
             i += delta

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    start_stop_service_node = MoveServiceServer(cmd_vel_topic="/cmd_vel", timer_period = 1.0)
    calculations_service_node = CalculationsServiceServer()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(start_stop_service_node)
    executor.add_node(calculations_service_node)
        
    try:
        executor.spin()
    finally:
        executor.shutdown()
        start_stop_service_node.destroy_node()
        calculations_service_node.destroy_node()
            
    
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()