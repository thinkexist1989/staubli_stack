#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItIkDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('moveit_ik_demo')

        arm = moveit_commander.MoveGroupCommander('arm')

        end_effector_link = arm.get_end_effector_link()

        reference_frame = 'base_link'

        arm.set_pose_reference_frame(reference_frame)

        arm.allow_replanning(True)

        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)

        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)

        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0
        target_pose.pose.position.y = 0.213868
        target_pose.pose.position.z = 0.520436
        target_pose.pose.orientation.x = 0.911822
        target_pose.pose.orientation.y = -0.0269758
        target_pose.pose.orientation.z = 0.285694
        target_pose.pose.orientation.w = -0.293653

        arm.set_start_state_to_current_state()

        arm.set_pose_target(target_pose, end_effector_link)

        traj = arm.plan()

        arm.execute(traj)
        rospy.sleep(1)

        arm.shift_pose_target(1, -0.05, end_effector_link)
        arm.go()
        rospy.sleep(1)

        arm.shift_pose_target(3,-1.57, end_effector_link)
        arm.go()
        rospy.sleep(1)

        arm.set_named_target('home')
        arm.go()

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()