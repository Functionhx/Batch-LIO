import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('batch_lio')
    default_cfg = os.path.join(pkg_share, 'config', 'avia.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz_cfg', 'loam_livox.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_cfg),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='batch_lio',
            executable='batchlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
