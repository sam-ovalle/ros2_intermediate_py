import argparse
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from std_msgs.msg import String
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
# Time
from rclpy.clock import ClockType
from rclpy.time import Time

from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy

from sensor_msgs.msg import LaserScan

class SubscriberDDS(Node):

    def __init__(self, qos_profile_subscriber):
        # Here we have the class constructor
        # call super() in the constructor in order to initialize the Node object
        # the parameter we pass is the node name
        super().__init__('subscriber_dds_obj')

        self.id = 0
        self.id_next = -1
        self.lost_messages_counter = 0

        # create the subscriber object
        self.subscriber= self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile=qos_profile_subscriber)

        # prevent unused variable warning
        self.subscriber

        

    def listener_callback(self, msg):
        # print the log info in the terminal        
        self.get_logger().info('I receive: "%s"' % str(msg))


    

def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-history',
        type=str,
        choices=['keep_last','keep_all'],
        default='keep_last',
        help='Select Policy for history, use ros2 run dds_tests_pkg subscriber_scan_qos -history keep_last|keep_all')
    parser.add_argument(
        '-depth',
        type=int,
        default=1,
        help='Select Policy for depth, use ros2 run dds_tests_pkg subscriber_scan_qos -depth 42')
    parser.add_argument(
        '-reliability',
        type=str,
        choices=['best_effort','reliable'],
        default='best_effort',
        help='Select Policy for reliability, use ros2 run dds_tests_pkg subscriber_scan_qos -reliability best_effort|reliable')
    parser.add_argument(
        '-durability',
        type=str,
        choices=['volatile','transient_local'],
        default='volatile',
        help='Select Policy for durability, use ros2 run dds_tests_pkg subscriber_scan_qos -durability volatile|transient_local')
    parser.add_argument(
        '-lifespan',
        type=float,
        default=0,
        help='Select Policy for lifespan, use ros2 run dds_tests_pkg subscriber_scan_qos -lifespan 2')
    parser.add_argument(
        '-deadline',
        type=float,
        default=0,
        help='Select Policy for reliability, use ros2 run dds_tests_pkg subscriber_scan_qos -deadline 2')
    parser.add_argument(
        '-liveliness_lease_duration',
        type=float,
        default=0,
        help='Select Policy for reliability, use ros2 run dds_tests_pkg subscriber_scan_qos -liveliness_lease_duration 2')
    parser.add_argument(
        '-liveliness',
        type=str,
        choices=['manual_by_topic','automatic'],
        default='automatic',
        help='Select Policy for reliability, use ros2 run dds_tests_pkg subscriber_scan_qos -liveliness manual_by_topic|automatic')
    return parser    
        
            
def main(args=None):

    # Lets parse the arguments
    parser = get_parser()
    parsed_args = parser.parse_args()

    # Configuration variables
    history = parsed_args.history
    reliability = parsed_args.reliability
    durability = parsed_args.durability
    lifespan = parsed_args.lifespan
    deadline = parsed_args.deadline
    liveliness = parsed_args.liveliness
    liveliness_lease_duration = parsed_args.liveliness_lease_duration
    depth = parsed_args.depth

    print("######## - INPUT ARGS -#########")
    print("history="+str(history))
    print("reliability="+str(reliability))
    print("durability="+str(durability))
    print("lifespan="+str(lifespan))
    print("deadline="+str(deadline))
    print("liveliness="+str(liveliness))
    print("liveliness_lease_duration="+str(liveliness_lease_duration))
    print("depth="+str(depth))
    print("################################")

    # Custom QoS
    # We have to init wth depth and history
    # Otherwise Error: rclpy.qos.InvalidQoSProfileException: ('Invalid QoSProfile', 'History and/or depth settings are required.')
    # We set history to keep last, and then deoth will set how many will be stored
    # Options: QoSHistoryPolicy.KEEP_LAST, or QoSHistoryPolicy.KEEP_ALL
    history_qos_value = None
    if history=="keep_all":
        history_qos_value = QoSHistoryPolicy.KEEP_ALL
    else:
        history_qos_value = QoSHistoryPolicy.KEEP_LAST

    qos_profile_subscriber = QoSProfile(history=history_qos_value, depth=depth)
    

    # Options  QoSDurabilityPolicy.VOLATILE, QoSDurabilityPolicy.TRANSIENT_LOCAL,
    if durability=="transient_local":
        qos_profile_subscriber.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    else:
        qos_profile_subscriber.durability = QoSDurabilityPolicy.VOLATILE

    # TODO: Lifespann is not considered in subscribers, only in publishers
    # https://github.com/ros2/rclpy/pull/523/commits/67fd59136e6b2a73849ab59af268a374fb46795b
    qos_profile_subscriber.lifespan = Duration(seconds=lifespan)

    qos_profile_subscriber.deadline = Duration(seconds=deadline)

    # Options QoSLivelinessPolicy.MANUAL_BY_TOPIC, QoSLivelinessPolicy.AUTOMATIC
    if liveliness=="manual_by_topic":
        qos_profile_subscriber.liveliness = QoSLivelinessPolicy.MANUAL_BY_TOPIC
    else:
        qos_profile_subscriber.liveliness = QoSLivelinessPolicy.AUTOMATIC

    qos_profile_subscriber.liveliness_lease_duration = Duration(seconds=liveliness_lease_duration)

    # Options: QoSReliabilityPolicy.RELIABLE, QoSReliabilityPolicy.BEST_EFFORT
    if reliability=="reliable":
        qos_profile_subscriber.reliability = QoSReliabilityPolicy.RELIABLE
    else:
        qos_profile_subscriber.reliability = QoSReliabilityPolicy.BEST_EFFORT

    

    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    subscriber_dds_obj = SubscriberDDS(qos_profile_subscriber)
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(subscriber_dds_obj)
    # Explicity destroy the node
    subscriber_dds_obj.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()