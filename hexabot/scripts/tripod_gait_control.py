#! /usr/bin/env python3

import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math

def IK(x0):
    J1L = 18.75 # length [mm] of coxa joint (coxa to femur joint)
    J2L = 65 # length [mm] of femur joint (femur to tibia joint)
    J3L = 102.5+12.99 # length [mm] of tibia joint (tibia to foot)

    # User-defined separation distance Y0 from body (hence minus J1L) to foot 
    Y0 = 90 - J1L

    #----- Point 1 ---
    # User-defined height H [mm] of body from ground
    H1 = 75
    
    # Inverse kinematics
    L1 = math.sqrt(H1**2 + Y0 **2)
    J21 = math.acos((L1**2 + J2L**2 - J3L**2)/(2*L1*J2L)) - math.atan(H1/Y0)
    J31 = math.pi - math.acos((J2L**2 + J3L**2 - L1**2)/(2*J2L*J3L))

    # J1 angles for initialization are chosen arbitrarily. 
    J11 = math.atan((-x0/2)/(Y0))
    #-----------------
    
    #----- Point 2 ---
    H2 = H1 - 35
    # Inverse kinematics
    L2 = math.sqrt(H2**2 + Y0 **2)
    J22 = math.acos((L2**2 + J2L**2 - J3L**2)/(2*L2*J2L)) - math.atan(H2/Y0)
    J32 = math.pi - math.acos((J2L**2 + J3L**2 - L2**2)/(2*J2L*J3L))

    J12 = 0.0
    #-----------------

    #----- Point 3 ---
    H3 = H1
    # Inverse kinematics
    L3 = math.sqrt(H3**2 + Y0 **2)
    J23 = math.acos((L3**2 + J2L**2 - J3L**2)/(2*L3*J2L)) - math.atan(H3/Y0)
    J33 = math.pi - math.acos((J2L**2 + J3L**2 - L3**2)/(2*J2L*J3L))

    J13 = math.atan((x0/2+math.tan(J12)*Y0)/(Y0))
    #-----------------

    #----- Point 4 ---
    H4 = H1
    # Inverse kinematics
    L4 = math.sqrt(H4**2 + Y0 **2)
    J24 = math.acos((L4**2 + J2L**2 - J3L**2)/(2*L4*J2L)) - math.atan(H4/Y0)
    J34 = math.pi - math.acos((J2L**2 + J3L**2 - L4**2)/(2*J2L*J3L))

    J14 = 0.0
    #-----------------

    point1_pos = [J11, J21, J31]

    point2_pos = [J12, J22, J32]

    point3_pos = [J13, J23, J33]

    point4_pos = [J14, J24, J34]

    return [point1_pos, point2_pos, point3_pos, point4_pos]

def initial_pose(J1):
    # J1 angles for initialization are chosen arbitrarily. 

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

    # angle definition in coxa, femur, tibia triplets for each leg 1-6
    angles = [0.0] * 18
    angles[0:3] = [J1 * 1, -J2, -J3] # leg 1
    angles[3:6] = [J1 * 0, -J2, -J3] # leg 2
    angles[6:9] = [J1 * -1, -J2, -J3] # leg 3
    angles[9:12] = [J1 * 1, -J2, -J3] # leg 4
    angles[12:15] = [J1 * 0, -J2, -J3] # leg 5
    angles[15:18] = [J1 * -1, -J2, -J3] # leg 6

    return angles

class SteeringActionClient(Node):

    def __init__(self):
        super().__init__('single_leg_control_actionclient')
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory'
        )
    
    def send_goal(self, x0):
        goal_msg = FollowJointTrajectory.Goal()


        # joint_names = [f"leg{leg_no}_coxa",
        #                f"leg{leg_no}_femur",
        #                f"leg{leg_no}_tibia"]
        
        joint_names = ["leg1_coxa",
                       "leg1_femur",
                       "leg1_tibia",
                       "leg2_coxa",
                       "leg2_femur",
                       "leg2_tibia",
                       "leg3_coxa",
                       "leg3_femur",
                       "leg3_tibia",
                       "leg4_coxa",
                       "leg4_femur",
                       "leg4_tibia",
                       "leg5_coxa",
                       "leg5_femur",
                       "leg5_tibia",
                       "leg6_coxa",
                       "leg6_femur",
                       "leg6_tibia",
        ]


        P1, P2, P3, P4 = IK(x0)

        points = []

        point0 = JointTrajectoryPoint()
        point0.positions = initial_pose(J1 = 0.0)
        point0.time_from_start = Duration(seconds = 0, nanoseconds = 0).to_msg()
        points.append(point0)

        point1 = JointTrajectoryPoint()
        
        angles = [0.0] * 18
        angles[0:3] = [P1[0], -P1[1], -P1[2]]     # leg 1
        angles[3:6] = [P3[0], -P3[1], -P3[2]]  # leg 2
        angles[6:9] = [P1[0], -P1[1], -P1[2]]     # leg 3
        angles[9:12] = [-P3[0], -P3[1], -P3[2]]  # leg 4
        angles[12:15] = [-P1[0], -P1[1], -P1[2]]  # leg 5
        angles[15:18] = [-P3[0], -P3[1], -P3[2]] # leg 6

        point1.positions = angles

        point1.time_from_start = Duration(seconds = 1, nanoseconds = 0).to_msg()


        point2 = JointTrajectoryPoint()

        angles = [0.0] * 18
        angles[0:3] = [P2[0], -P2[1], -P2[2]]     # leg 1
        angles[3:6] = [P4[0], -P4[1], -P4[2]]  # leg 2
        angles[6:9] = [P2[0], -P2[1], -P2[2]]     # leg 3
        angles[9:12] = [-P4[0], -P4[1], -P4[2]]  # leg 4
        angles[12:15] = [-P2[0], -P2[1], -P2[2]]  # leg 5
        angles[15:18] = [-P4[0], -P4[1], -P4[2]] # leg 6

        point2.positions = angles

        point2.time_from_start = Duration(seconds = 2, nanoseconds = 0).to_msg()

        point3 = JointTrajectoryPoint()

        angles = [0.0] * 18
        angles[0:3] = [P3[0], -P3[1], -P3[2]]     # leg 1
        angles[3:6] = [P1[0], -P1[1], -P1[2]]  # leg 2
        angles[6:9] = [P3[0], -P3[1], -P3[2]]     # leg 3
        angles[9:12] = [-P1[0], -P1[1], -P1[2]]  # leg 4
        angles[12:15] = [-P3[0], -P3[1], -P3[2]]  # leg 5
        angles[15:18] = [-P1[0], -P1[1], -P1[2]] # leg 6

        point3.positions = angles

        point3.time_from_start = Duration(seconds = 3, nanoseconds = 0).to_msg()


        point4 = JointTrajectoryPoint()

        angles = [0.0] * 18
        angles[0:3] = [P4[0], -P4[1], -P4[2]]     # leg 1
        angles[3:6] = [P2[0], -P2[1], -P2[2]]  # leg 2
        angles[6:9] = [P4[0], -P4[1], -P4[2]]     # leg 3
        angles[9:12] = [-P2[0], -P2[1], -P2[2]]  # leg 4
        angles[12:15] = [-P4[0], -P4[1], -P4[2]]  # leg 5
        angles[15:18] = [-P2[0], -P2[1], -P2[2]] # leg 6


        point4.positions = angles

        point4.time_from_start = Duration(seconds = 4, nanoseconds = 0).to_msg()

        point_positions = [point1.positions,
                           point2.positions,
                           point3.positions,
                           point4.positions]

        num_cycles = 5
        n = 1 # time counter
        for i in range(1,(num_cycles*4) + 1):
            new_point = JointTrajectoryPoint()
            new_point.positions = point_positions[(i % 4) - 1]
            new_point.time_from_start = Duration(seconds = n, nanoseconds = 0).to_msg()
            points.append(new_point)
            n += 1
        
        end_point = JointTrajectoryPoint()
        end_point.positions = initial_pose(J1 = 0.0)
        end_point.time_from_start = Duration(seconds = n + 1, nanoseconds = 0).to_msg()
        points.append(end_point)

        goal_msg.goal_time_tolerance = Duration(seconds = 1, nanoseconds = 0).to_msg()

        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback = self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: ' + str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback



def main(args = None):

    rclpy.init()

    action_client = SteeringActionClient()
    
    x0 = float(sys.argv[1])

    future = action_client.send_goal(x0)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()