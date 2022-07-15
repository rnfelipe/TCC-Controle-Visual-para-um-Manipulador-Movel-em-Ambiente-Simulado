#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "Arm"
group = moveit_commander.MoveGroupCommander(group_name)

#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)
# We can get the name of the reference frame for this robot:

#planning_frame = group.get_planning_frame()
#print("============ Reference frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
#eef_link = group.get_end_effector_link()
#print("============ End effector: %s" % eef_link)

# We can get a list of all the groups in the robot:
#group_names = robot.get_group_names()
#print("============ Robot Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
#print("============ Printing robot state")
#print(robot.get_current_state())
#print("")
#current_pose = group.get_current_pose()

#print("============ Printing current EEf pose")
#print(current_pose)

pose_goal = geometry_msgs.msg.Pose()

def homePose():
    pose_goal.orientation.x = -0.019
    pose_goal.orientation.y = -0.008
    pose_goal.orientation.z = 0.966
    pose_goal.orientation.w = 0.254
    pose_goal.position.x = -0.03
    pose_goal.position.y = -0.11
    pose_goal.position.z = 0.60

    return pose_goal

def preGraspingPose():
    pose_goal.orientation.x = -0.030 # 0.01
    pose_goal.orientation.y = 0.021 # -0.05
    pose_goal.orientation.z = -0.999 # 0.99
    pose_goal.orientation.w = 0.001 # 0.07
    pose_goal.position.x = 0.65 
    pose_goal.position.y = -0.05
    pose_goal.position.z = 0.66

    return pose_goal

def pubHomePose():
    next_pose = homePose()
    group.set_pose_target(next_pose)
    plan2 = group.plan()
    group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    #group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    #group.execute(plan, wait=True)


def pubPreGraspingPose():
    next_pose = preGraspingPose()
    group.set_pose_target(next_pose)
    
    plan2 = group.plan()   
    group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    #group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    #group.execute(plan, wait=True)

#pubHomePose()
#print("Cheguei a pose Home")
pubPreGraspingPose()
print("Chegueeeeei")

