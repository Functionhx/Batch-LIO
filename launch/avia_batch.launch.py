import os

# Headless batch-LIO launch on Livox AVIA bags (ROS2 port of scripts/avia_batch.launch).
# Batch params (batch_dt / batch_omp / batch_deskew / pub_hifreq) default to the values in
# config/avia.yaml. To sweep them, prefer the scripts/run_lio.sh harness, which uses
# `ros2 run batch_lio batchlio_mapping --ros-args --params-file <cfg> -p batch_dt:=...`
# (the `-p key:=value` form infers the correct parameter type).
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('batch_lio'), 'config', 'avia.yaml')

    return LaunchDescription([
        Node(
            package='batch_lio',
            executable='batchlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[cfg],
        ),
    ])
