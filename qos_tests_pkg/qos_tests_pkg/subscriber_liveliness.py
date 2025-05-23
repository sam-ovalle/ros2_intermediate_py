import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.qos import QoSProfile
from rclpy.qos import QoSLivelinessPolicy

from rclpy.duration import Duration

import time

POLICY_MAP = {
    'AUTOMATIC': QoSLivelinessPolicy.AUTOMATIC,
    'MANUAL_BY_TOPIC': QoSLivelinessPolicy.MANUAL_BY_TOPIC,
}


class SubscriberQoS(Node):

    def __init__(self, qos_profile, node_name="publisher_qos_obj"):

        super().__init__(node_name)

        rclpy.logging.set_logger_level(
            node_name, rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = SubscriptionEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb,
            liveliness=self.liveliness_clb)

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

    def liveliness_clb(self, event):
        """
        # https://docs.ros2.org/dashing/api/rmw/types_8h_source.html
        rmw_liveliness_changed_status_t
        Liveliness triggered
            int32_t alive_count;
            int32_t not_alive_count;
            int32_t alive_count_change;
            int32_t not_alive_count_change;
        """
        self.get_logger().error("SUBSCRIBER::: Liveliness Triggered !")
        self.get_logger().error("alive_count="+str(event.alive_count))
        self.get_logger().error("not_alive_count="+str(event.not_alive_count))
        self.get_logger().error("alive_count_change="+str(event.alive_count_change))
        self.get_logger().error("not_alive_count_change="+str(event.not_alive_count_change))
        self.get_logger().error("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")


def get_parser():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '-liveliness_lease_duration',
        type=int,
        default=2000,
        help='Select Policy for liveliness_lease_duration in milliseconds, use ros2 run qos_tests_pkg name_of_exe -liveliness_lease_duration 3000')

    parser.add_argument(
        '--policy', type=str, choices=POLICY_MAP.keys(), default='AUTOMATIC',
        help='The Liveliness policy type. AUTOMATIC|MANUAL_BY_TOPIC')

    return parser


def main(args=None):

    parser = get_parser()
    parsed_args = parser.parse_args()

    liveliness_lease_duration = Duration(
        seconds=parsed_args.liveliness_lease_duration / 1000.0)

    liveliness_policy = POLICY_MAP[parsed_args.policy]

    qos_profile_subscriber = QoSProfile(
        depth=1,
        liveliness=liveliness_policy,
        liveliness_lease_duration=liveliness_lease_duration)

    rclpy.init(args=args)
    sub_qos_obj = SubscriberQoS(qos_profile=qos_profile_subscriber,
                                node_name="subscriber_liveliness")

    rclpy.spin(sub_qos_obj)
    sub_qos_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # ros2 run qos_tests_pkg subscriber_liveliness_exe -liveliness_lease_duration 450 --policy MANUAL_BY_TOPIC
    main()