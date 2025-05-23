# import the SetBool module from std_servs service interface
from std_srvs.srv import SetBool
# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the ROS2 python client libraries
import rclpy
from rclpy.node import Node
import time
# Added for Multithreading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class ServiceServer(Node):

    def __init__(self, cmd_vel_topic="/cmd_vel", timer_period = 0.5):

        # Multithreading 
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

            self.cmd.linear.x = 1.0
            self.cmd.angular.z =0.0
            
            response.success = True
            response.message = 'GOING FORWARD'

        self.wait_for_sec(15.0)
        self.get_logger().warning("Processing Service Server Message...DONE")
        # return the response parameters
        return response
    
    def wait_for_sec(self, wait_sec, delta=0.1):
        i = 0
        while i < wait_sec :
             self.get_logger().info("..."+str(i))
             time.sleep(delta)
             i += delta

    def timer_callback(self):
        self.publisher_.publish(self.cmd)
        # Display the message on the console
        self.print_cmd()

    def print_cmd(self):
        self.get_logger().info("[ X="+str(self.cmd.linear.x)+", A="+str(self.cmd.angular.z)+" ]")


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    start_turn_service_node = ServiceServer(cmd_vel_topic="/cmd_vel", timer_period = 1.0)
    # pause the program execution, waits for a request to kill the node (ctrl+c)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(start_turn_service_node)
        
    try:
        executor.spin()
    finally:
        executor.shutdown()
        start_turn_service_node.destroy_node()
            
    
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()