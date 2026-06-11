#!/usr/bin/env python3
"""
Sends the xArm6 to a non-singular ready pose at startup, then exits.
Publishes directly to the joint trajectory controller.

xArm6 joint limits (rad):
  joint1: ±3.11   joint2: -2.059..+2.094
  joint3: -3.11..+0.192   joint4: ±3.11
  joint5: -1.693..+3.11   joint6: ±3.11

Ready pose [0, -1.0, 0, 0, 1.0, 0] places the arm in an elbow-forward
configuration well away from joint limits and kinematic singularities.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


#                  j1    j2    j3   j4   j5   j6
READY_POSE_RAD = [0.0, -1.0,  0.0, 0.0, 1.0, 0.0]
JOINT_NAMES    = [f'joint{i}' for i in range(1, 7)]
MOVE_TIME_SEC  = 3


class InitialPose(Node):
    def __init__(self):
        super().__init__('initial_pose')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/xarm6_traj_controller/joint_trajectory',
            1,
        )
        self._sent = False
        # Wait for the controller to finish activating before sending
        self.create_timer(1.5, self.send_goal)

    def send_goal(self):
        if self._sent:
            return
        self._sent = True

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = READY_POSE_RAD
        pt.time_from_start = Duration(sec=MOVE_TIME_SEC)
        msg.points = [pt]

        self.pub.publish(msg)
        self.get_logger().info(f'Sent ready pose: {READY_POSE_RAD}')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
