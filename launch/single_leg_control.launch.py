from launch import LaunchDescription

import launch.actions
from launch_ros.actions import Node

def generate_launch_description():
    
    node_single_leg_control = Node(
        package = 'my_package',
        executable = 'single_leg_control.py',
        output = 'screen',
        arguments = ["5",       # leg number
                     "0.3",    # coxa angle
                     "-0.3",    # femur angle
                     "-0.3",    # tibia angle
        ]
    )

    return LaunchDescription([
        node_single_leg_control
        ])
