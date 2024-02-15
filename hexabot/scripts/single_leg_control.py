#! /usr/bin/env python3

import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class SteeringActionClient(Node):

    def __init__(self):
        super().__init__('single_leg_control_actionclient')
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory'
        )
    
    def send_goal(self, leg_no, angle_coxa, angle_femur, angle_tibia):
        goal_msg = FollowJointTrajectory.Goal()


        # joint_names = [f"leg{leg_no}_coxa",
        #                f"leg{leg_no}_femur",
        #                f"leg{leg_no}_tibia"]
        
        joint_names = [f"leg1_coxa",
                       f"leg1_femur",
                       f"leg1_tibia",
                       f"leg2_coxa",
                       f"leg2_femur",
                       f"leg2_tibia",
                       f"leg3_coxa",
                       f"leg3_femur",
                       f"leg3_tibia",
                       f"leg4_coxa",
                       f"leg4_femur",
                       f"leg4_tibia",
                       f"leg5_coxa",
                       f"leg5_femur",
                       f"leg5_tibia",
                       f"leg6_coxa",
                       f"leg6_femur",
                       f"leg6_tibia",
        ]
        
        
        points = []
        point1 = JointTrajectoryPoint()
        # point1.positions = [0.0, 0.0, 0.0,
        #                     0.2, -1.1, -2.0,
        #                     -0.2, -1.1, -2.0,
        #                     0.2, -1.1, -2.0,
        #                     -0.2, -1.1, -2.0,
        #                     0.0, 0.0, 0.0]
        point1.positions = [0.0] * len(joint_names)

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds = 3.0, nanoseconds = 0).to_msg()
    
        # According to what joints correspond to leg_no -> change the coxa, femur, tibia angles of that joint
        new_position = list(point1.positions)
        new_position[3*leg_no - 3 : 3*leg_no] = [angle_coxa, angle_femur, angle_tibia]
        point2.positions = new_position

        # point3 = JointTrajectoryPoint()
        # point3.time_from_start = Duration(seconds = 0.6, nanoseconds = 0).to_msg()

        # new_position = list(point1.positions)
        # new_position[3*leg_no - 3 : 3*leg_no] = [2*angle_coxa, -angle_femur, -angle_tibia]
        # point3.positions = new_position

        # points.append(point1)
        points.append(point2)
        # points.append(point3)

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

    # Specify which leg by leg number
    leg_no = int(sys.argv[1])

    # Specify angles[rad] in order: coxa > femur > tibia
    angles = [float(sys.argv[2]), 
              float(sys.argv[3]), 
              float(sys.argv[4])
    ]

    future = action_client.send_goal(leg_no = leg_no,
                                     angle_coxa = angles[0],
                                     angle_femur = angles[1],
                                     angle_tibia = angles[2])

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()



        
