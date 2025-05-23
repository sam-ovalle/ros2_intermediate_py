import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from std_msgs.msg import String
import random


class PublisherDDS(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the Node name
        super().__init__('publisher_dds_obj')
        # create the publisher object
        # in this case, the publisher will publish on /cmd_vel topic with a queue size of one message.
        self.publisher_ = self.create_publisher(String, '/dds_test', 1)
        # This is the Unique id for each of the messages that will be sent
        self.msgs_id = 0
        #self.current_time = self.get_clock().now()
        self.current_time_s = 0
        self.current_time_ns = 0
        # define the timer period for 0.5 seconds
        timer_period = 0.0
        # create a timer sending two parameters:
        # - the duration between two Callbacks (0.5 seconds)
        # - the timer function (timer_callback)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # here you have the Callback method
        # create a Twist message
        msg = String()
        test_time = self.get_clock().now()
        self.current_time_s, self.current_time_ns = test_time.seconds_nanoseconds()
        time_str = str(self.current_time_s)+","+str(self.current_time_ns)
        dds_msg_str = str(self.msgs_id)+":"+time_str
        msg.data = dds_msg_str
        # publish the message to the topic
        self.publisher_.publish(msg)
        # display the message on the console
        self.get_logger().info('Publishing: "%s"' % msg)
        
        self.msgs_id += 1
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the Node constructor
    publisher_dds_obj = PublisherDDS()
    # pause the program execution, waits for a request to kill the Node (ctrl+c)
    rclpy.spin(publisher_dds_obj)
    # Explicity destroy the Node
    publisher_dds_obj.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()