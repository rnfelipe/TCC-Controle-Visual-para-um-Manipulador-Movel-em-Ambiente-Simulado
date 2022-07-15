#!/usr/bin/env python

import sys
import copy
from weakref import ref
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist, Point

referencePosition = Point()
referenceOrientation = Point()
tagDetected = False
tagID = 1
positionX = 0.0
positionY = 0.0
positionZ = 0.0



moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial')

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "Arm"
group = moveit_commander.MoveGroupCommander(group_name)
group_all = moveit_commander.MoveGroupCommander("All")


#display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,queue_size=20)
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
    pose_goal.orientation.x = 0.035 # -0.030
    pose_goal.orientation.y = 0.004 # 0.021
    pose_goal.orientation.z = -0.999
    pose_goal.orientation.w = 0.006 # 0.001
    pose_goal.position.x = 0.648 # 0.65 
    pose_goal.position.y = -0.05 # -0.05
    pose_goal.position.z = 0.530 # 0.66

    return pose_goal

def posGraspingPose():
    pose_goal.orientation.x = 0.035
    pose_goal.orientation.y = 0.004
    pose_goal.orientation.z = -0.999
    pose_goal.orientation.w = 0.006
    pose_goal.position.x = 0.648 
    pose_goal.position.y = -0.050
    pose_goal.position.z = 0.707

    return pose_goal

def placePose():
    pose_goal.orientation.x = -0.016
    pose_goal.orientation.y = -0.029
    pose_goal.orientation.z = 0.712
    pose_goal.orientation.w = 0.700
    pose_goal.position.x = 0.305 
    pose_goal.position.y = -0.605
    pose_goal.position.z = 0.707

    # Juntas [1.4913355596322893, -3.3164991264665318, -0.3058994167881517, 2.801326972958986, -1.7447371255902149, 1.136473878039646]


    return pose_goal

def pubPlacePose():
    next_pose = placePose()
    group.set_pose_target(next_pose)
    plan2 = group.plan()   
    group.go(wait=True)
    group.stop()

def pubGraspingPose():
    pubOpenFingers()    
    graspingPose = preGraspingPose()

    graspingPose.position.y -= (referencePosition.x + 0.03)
    graspingPose.position.z -= (referencePosition.y - 0.09)
    group.set_pose_target(graspingPose)
    plan2 = group.plan()
    group.go(wait=True)
    group.stop()

    #graspingPose.position.z -= (referencePosition.y - 0.17) # OK
    #group.set_pose_target(graspingPose)
    #plan2 = group.plan()
    #group.go(wait=True)
    #group.stop()

    graspingPose.position.x += (referencePosition.z - 0.24)
    group.set_pose_target(graspingPose)
    plan2 = group.plan()
    group.go(wait=True)
    group.stop()

    #graspingPose.position.z -= (referencePosition.y - 0.117)
    #group.set_pose_target(graspingPose)
    #plan2 = group.plan()
    #group.go(wait=True)
    #group.stop()

    rospy.sleep(1)
    pubCloseFingers()

    graspingPose.position.z += 0.15
    #graspingPose.position.x -= 0.30
    group.set_pose_target(graspingPose)
    plan2 = group.plan()
    group.go(wait=True)
    group.stop()

    group.clear_pose_targets()

def pubOpenFingers():
    group_variable_values = group_all.get_current_joint_values()
    group_variable_values[6] = 0.02
    group_variable_values[7] = 0.02
    group_variable_values[8] = 0.02

    group_all.set_joint_value_target(group_variable_values)

    plan2 = group_all.plan()
    group_all.go(wait=True)
    group_all.stop()

    #rospy.sleep(3)

def pubCloseFingers():
    group_variable_values = group_all.get_current_joint_values()
    group_variable_values[6] = 0.53
    group_variable_values[7] = 0.53
    group_variable_values[8] = 0.53

    group_all.set_joint_value_target(group_variable_values)

    plan2 = group_all.plan()
    group_all.go(wait=True)
    group_all.stop()

    rospy.sleep(2)

def pubHomePose():
    next_pose = homePose()
    group.set_pose_target(next_pose)
    plan2 = group.plan()
    group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
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


def getReference(msg):
    global referencePosition
    global referenceOrientation
    global tagDetected
    global tagID

    tagDetected = msg.detections[0]
    tagID = tagDetected.id[0]
    #print('IDs = {}' .format(len(msg.detections)))
    #print('Frame = {}' .format(tagDetected.pose.header.frame_id))


    if ((tagDetected) and (tagDetected.pose.header.frame_id == "camera_realsense_gazebo")):
        referencePosition = tagDetected.pose.pose.pose.position
        #print('Position = {}' .format(referencePosition))
        referenceOrientation = tagDetected.pose.pose.pose.orientation
    
    #print('Position = {}' .format(referencePosition))
    '''else:
        print("PASSEEEEI DIRETOOOO")
        referencePosition = Point()
        referenceOrientation = Point()
    '''

rospy.init_node('move_group_python_interface_tutorial')
marker = rospy.Subscriber("tag_detections", AprilTagDetectionArray, getReference)

r = rospy.Rate(100)

while not rospy.is_shutdown():
    positionX = referencePosition.z
    positionY =  referencePosition.x
    positionZ =  referencePosition.y
    
    #print(group_variable_values)
    #pubOpenFingers()
    #pubPreGraspingPose()
    pubPlacePose()
    #pubHomePose()
    rospy.sleep(3)
    print("Juntas")
    print(group.get_current_joint_values())
    print("Pose")
    print(group.get_current_pose())
    while (referencePosition.z == 0.0):
        continue

    #pubGraspingPose()
    #rospy.sleep(2)
    #pubPreGraspingPose()
    print("Chegueeeeei")



    break

    r.sleep()

#pubHomePose()
#print("Cheguei a pose Home")
#pubGraspingPose()
#print("Chegueeeeei")

