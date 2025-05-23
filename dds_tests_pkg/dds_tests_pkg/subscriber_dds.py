import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from std_msgs.msg import String
# import QoS library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
# Time
from rclpy.clock import ClockType
from rclpy.time import Time

class SubscriberDDS(Node):

    def __init__(self):
        # here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the Node name
        super().__init__('subscriber_dds_obj')

        self.id = 0
        self.id_next = -1
        self.lost_messages_counter = 0

        # create the subscriber object
        self.subscriber= self.create_subscription(
            String,
            '/dds_test',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) #is the most used to read LaserScan data and some sensor data.
        # prevent unused variable warning
        self.subscriber

        

    def listener_callback(self, msg):
        # print the log info in the terminal
        
        # self.get_logger().info('I receive: "%s"' % str(msg))
        # save the received data
        self.process_dds_test_data(msg.data)

    def process_dds_test_data(self,data):
        """
        The data has the format ID:Seconds,NanoSeconds
        return:
        ID
        TIME Object
        """
        array = data.split(":")
        self.id = int(array[0])
        if self.id == 0:
            self.id_next = 0


        if self.id == self.id_next:
            self.get_logger().warning("MESSAGE OK: ID="+str(self.id)+" ,Next ID="+str(self.id_next)+", TOTAL MSG LOST="+str(self.lost_messages_counter))
        else:
            if self.id_next != -1:
                delta_messages_lost = self.id - self.id_next
                self.lost_messages_counter += delta_messages_lost
                self.get_logger().error("Message LOST: ID="+str(self.id)+" ,Next ID="+str(self.id_next)+", DELTA MSG LOST="+str(delta_messages_lost)+", TOTAL MSG LOST="+str(self.lost_messages_counter))
                
            else:
                # This is to avoid error in the first init message
                pass
        
        self.id_next = self.id + 1

        seconds = int(array[1].split(",")[0])
        nano_seconds = int(array[1].split(",")[1])
        time_obj = Time(seconds=seconds, 
                        nanoseconds=nano_seconds,
                        clock_type=ClockType.ROS_TIME)
        time_now_obj = self.get_clock().now()
        delta = time_now_obj - time_obj

        self.get_logger().info("DELTA Between PUB and SUB ="+str(delta))
        
        
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the Node constructor
    subscriber_dds_obj = SubscriberDDS()
    # pause the program execution, waits for a request to kill the Node (ctrl+c)
    rclpy.spin(subscriber_dds_obj)
    # explicity destroy the Node
    subscriber_dds_obj.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()