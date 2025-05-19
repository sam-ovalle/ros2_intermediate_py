import os
import sys

from threading import Event
from threading import Thread
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    path_to_test = os.path.dirname(__file__)

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            executable=sys.executable,
            arguments=[os.path.join(path_to_test, 'dummy_laser.py')],
            name='testing_node_dummy_scan',
        ),

        launch_testing.actions.ReadyToTest()
    ])


class TestFixture(unittest.TestCase):

    def test_check_if_msgs_published(self, proc_output):
        rclpy.init()
        try:
            node = MakeTestNode('test_node')
            node.start_subscriber()
            msgs_received_flag = node.msg_event_object.wait(timeout=5.0)
            assert msgs_received_flag, 'ERROR in TEST: Did not get any message!'
        finally:
            rclpy.shutdown()


class MakeTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)
        self.msg_event_object = Event()

    def start_subscriber(self):
        # Create a subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/dummy_scan',
            self.subscriber_callback,
            10
        )

        # Add a spin thread
        self.ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def subscriber_callback(self, data):
        self.msg_event_object.set()