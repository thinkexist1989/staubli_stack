#!/usr/bin/env python

import rospy, sys
import moveit_commander

from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory
from copy import deepcopy

class MoveItCartesianDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        arm = moveit_commander.MoveGroupCommander('arm')

        arm.allow_replanning(True)

        arm.set_pose_reference_frame('base_link')

        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.1)

        end_effector_link = arm.get_end_effector_link()

        arm.set_named_target('cutting')
        arm.go()
        rospy.sleep(2)

        start_pose = arm.get_current_pose(end_effector_link).pose

        waypoints = []

        #waypoints.append(start_pose)

        wpose = deepcopy(start_pose)

        wpose.position.x -= 0.2
        # wpose.position.y += 0.2

        waypoints.append(deepcopy(wpose))

        wpose.position.z -= 0.1
        # wpose.position.y += 0.2

        waypoints.append(deepcopy(wpose))

        wpose.position.x -= 0.05
        wpose.position.y -= 0.15
        # wpose.position.z -= 0.15

        waypoints.append(deepcopy(wpose))

        waypoints.append(deepcopy(start_pose))

        fraction = 0.0
        maxtries = 100
        attempts = 0

        # arm.set_start_state_to_current_state()

        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.01, 0.0, True)
            attempts += 1
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after" + str(attempts) + " attempts...")
            
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm...")
            arm.execute(plan,wait=True)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planing failed with only" + str(fraction) + "success after" + str(maxtries) + "attemps.")

        # arm.set_named_target('home')
        # arm.go()
        # rospy.sleep(1)

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass

