from launch import LaunchDescription

import launch.actions
from launch_ros.actions import Node
import math

def generate_launch_description():

    J1L = 18.75 # length [mm] of coxa joint (coxa to femur joint)
    J2L = 65 # length [mm] of femur joint (femur to tibia joint)
    J3L = 102.5+12.99 # length [mm] of tibia joint (tibia to foot)

    # User-defined height H [mm] of body from ground
    H = 70
    # User-defined separation distance Y0 from body (hence minus J1L) to foot 
    Y0 = 90 - J1L

    # Inverse kinematics
    L = math.sqrt(H**2 + Y0 **2)
    J2 = math.acos((L**2 + J2L**2 - J3L**2)/(2*L*J2L)) - math.atan(H/Y0)
    J3 = math.pi - math.acos((J2L**2 + J3L**2 - L**2)/(2*J2L*J3L))

    # J1 angles for initialization are chosen arbitrarily. 
    J1 = 0.0

    # angle definition in coxa, femur, tibia triplets for each leg 1-6
    angles = [0.0] * 18
    angles[0:3] = [J1 * 1, -J2, -J3] # leg 1
    angles[3:6] = [J1 * 0, -J2, -J3] # leg 2
    angles[6:9] = [J1 * -1, -J2, -J3] # leg 3
    angles[9:12] = [J1 * 1, -J2, -J3] # leg 4
    angles[12:15] = [J1 * 0, -J2, -J3] # leg 5
    angles[15:18] = [J1 * -1, -J2, -J3] # leg 6

    node_multi_leg_control = Node(
        package = 'hexabot',
        executable = 'multi_leg_control.py',
        output = 'screen',
        arguments = [f"{angles}"]
    )

    return LaunchDescription([
        node_multi_leg_control
        ])
