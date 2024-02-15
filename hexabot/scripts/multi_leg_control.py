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
    
    def send_goal(self, angles):
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
        
        
        points = []
        point1 = JointTrajectoryPoint()
        # point1.positions = [0.0, 0.0, 0.0,
        #                     0.2, -1.1, -2.0,
        #                     -0.2, -1.1, -2.0,
        #                     0.2, -1.1, -2.0,
        #                     -0.2, -1.1, -2.0,
        #                     0.0, 0.0, 0.0]
        point1.positions = angles
        
        point1.time_from_start = Duration(seconds = 3.0, nanoseconds = 0).to_msg()

        points.append(point1)       

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
    
    # angles = sys.argv[1]
    angles = [float(i) for i in sys.argv[1][1:-1].split(',')]

    future = action_client.send_goal(angles)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()