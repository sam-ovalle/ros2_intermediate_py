import rclpy
from rclpy.node import Node
import time
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile

class SubscriberClass(Node):

    def __init__(self):
        super().__init__('sub_node')
        rclpy.logging.set_logger_level('sub_node', rclpy.logging.LoggingSeverity.DEBUG)
        
        self.group = ReentrantCallbackGroup()
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10, callback_group=self.group)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.group)
        self.timer = self.create_timer(0.5, self.timer_callback, callback_group=self.group)
        self.laser_msg = LaserScan()
        self.odom_msg = Odometry()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.publisher = PublisherClass()

    def odom_callback(self, msg):
        self.get_logger().debug("Odom CallBack")
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = self.euler_from_quaternion (orientation_list)

    def scan_callback(self, msg):
        self.get_logger().debug("Scan CallBack")
        self.laser_msg = msg

    def get_front_laser(self):
        return self.laser_msg.ranges[360]

    def get_yaw(self):
        return self.yaw

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to Euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def timer_callback(self):
        self.get_logger().info("Timer CallBack")
        try:
            self.get_logger().warning(">>>>>>>>>>>>>>RANGES Value="+str(self.laser_msg.ranges[360]))
            if not self.laser_msg.ranges[360] < 0.8:                
                self.publisher.move_straight()
            else:                
                self.publisher.stop_robot()
                self.publisher.rotate()
        except:
            pass

class PublisherClass(Node):

    def __init__(self, seconds_sleeping=10):
        super().__init__('pub_node')
        self._seconds_sleeping = seconds_sleeping
        rclpy.logging.set_logger_level('pub_node', rclpy.logging.LoggingSeverity.DEBUG)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()
    
    def stop_robot(self):
        self.get_logger().info("Ex2 MOVE STOP")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)

    def move_straight(self):
        self.get_logger().info("Ex2 MOVE STRAIGHT")
        self.cmd.linear.x = 0.15
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)
    
    def rotate(self):
        self.get_logger().info("Ex2 MOVE ROTATE")
        self.cmd.angular.z = -0.2
        self.cmd.linear.x = 0.0
        self.get_logger().info("PUBLISH COMMAND...")
        self.vel_pub.publish(self.cmd)        
        self.get_logger().info("PUBLISH COMMAND...FINISHED")
        self.get_logger().info("Ex2 Rotating for "+str(self._seconds_sleeping)+" seconds")
        for i in range(self._seconds_sleeping):
            self.get_logger().info("Ex2 SLEEPING=="+str(i)+" seconds")
            time.sleep(1)
        
        self.stop_robot()
    

def main(args=None):
    rclpy.init(args=args)
    try:
        subs_node = SubscriberClass()
        pub_node = PublisherClass()
        
        executor = SingleThreadedExecutor()
        executor.add_node(subs_node)
        executor.add_node(pub_node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            subs_node.destroy_node()
            pub_node.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()