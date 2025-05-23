import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy


class SubscriberQoS(Node):

    def __init__(self, qos_profile, node_name="subscriber_qos_obj"):

        super().__init__(node_name)

        rclpy.logging.set_logger_level(
            node_name,
            rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = SubscriptionEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb)

        self.subscriber = self.create_subscription(
            msg_type=String,
            topic='/qos_test',
            callback=self.listener_callback,
            qos_profile=qos_profile,
            event_callbacks=event_callbacks)

    def listener_callback(self, msg):
        self.get_logger().info("Data Received ="+str(msg.data))

    def incompatible_qos_clb(self, event):
        """
        This is the Callback that will be executed when the event of **Incompatible QoS** is
        triggered.
        """
        self.get_logger().error("SUBSCRIBER::: INCOMPATIBLE QoS Triggered!")
        self.get_logger().error(str(event.last_policy_kind))
        self.get_logger().error("############################")


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-durability',
        type=str,
        choices=['transient_local', 'volatile'],
        help='Select Policy for durability, use ros2 run qos_tests_pkg name_of_exe -durability transient_local|volatile')
    return parser


def main(args=None):

    parser = get_parser()
    parsed_args = parser.parse_args()

    # Depth of History is Compulsory
    qos_profile_subscriber = QoSProfile(depth=1)

    # Options  QoSDurabilityPolicy.VOLATILE, QoSDurabilityPolicy.TRANSIENT_LOCAL,
    durability = parsed_args.durability
    print(durability)
    if durability == "transient_local":
        qos_profile_subscriber.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    else:
        # Leave the one by default, which is VOLATILE
        pass

    rclpy.init(args=args)
    sub_qos_obj = SubscriberQoS(qos_profile_subscriber)
    rclpy.spin(sub_qos_obj)
    sub_qos_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()