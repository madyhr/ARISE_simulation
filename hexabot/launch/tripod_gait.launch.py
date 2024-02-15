from launch import LaunchDescription

import launch.actions
from launch_ros.actions import Node
import math

def generate_launch_description():

    x0 = 30 # step length [mm]

    node_tripod_gait_control = Node(
        package = 'hexabot',
        executable = 'tripod_gait_control.py',
        output = 'screen',
        arguments = [f"{x0}"]
    )

    return LaunchDescription([
        node_tripod_gait_control
        ])
