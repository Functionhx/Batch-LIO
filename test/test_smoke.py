"""Smoke test: launch batch_lio, play the converted Livox Avia bag, assert odometry.

Requires the converted smoke bag at ~/batch_lio_ws/bags/quick-shack (produced by
scripts/convert_bag.py from the ROS1 quick-shack bag).
"""
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node

BAG = os.path.expanduser('~/batch_lio_ws/bags/quick-shack')
TIMEOUT_S = 90.0
NEED_MESSAGES = 5


def generate_test_description():
    cfg = os.path.join(get_package_share_directory('batch_lio'), 'config', 'avia.yaml')
    mapping = launch_ros.actions.Node(
        package='batch_lio',
        executable='batchlio_mapping',
        name='laserMapping',
        parameters=[cfg],
        output='screen',
    )
    player = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'play', BAG, '--rate', '2.0'],
        output='screen',
    )
    return (
        launch.LaunchDescription([
            mapping,
            player,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'mapping': mapping, 'player': player},
    )


class TestOdometryPublishes(unittest.TestCase):
    def test_publishes_odometry(self, proc_output, mapping, player):
        rclpy.init()
        node = Node('smoke_listener')
        received = []
        node.create_subscription(
            Odometry, '/aft_mapped_to_init', lambda m: received.append(m), 10
        )
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        deadline = time.monotonic() + TIMEOUT_S
        while time.monotonic() < deadline and len(received) < NEED_MESSAGES:
            executor.spin_once(timeout_sec=0.5)
        node.destroy_node()
        rclpy.shutdown()
        self.assertGreaterEqual(
            len(received), 1, 'no odometry published on /aft_mapped_to_init within timeout'
        )
