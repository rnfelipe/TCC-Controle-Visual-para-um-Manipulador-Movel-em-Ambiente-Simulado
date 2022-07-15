#!/usr/bin/env python
import sys
import moveit_commander
from math import cos, sin
import rospy
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray

huskyReferencePosition = Point()
huskyReferenceOrientation = Point()
jacoReferencePosition = Point()

# Husky positions and orientation related to the tag
huskyPositionX = 0.0
huskyPositionY = 0.0
huskyOrientationYaw = 0.0

# Jaco positions related to the tag
jacoPositionX = 0.0
jacoPositionY = 0.0
jacoPositionZ = 0.0

# Positions and orientation references for navigation
refX = 0.5
refY = 0.0
refYaw = 0.0

error_tolerance = 0.1
error_tolerance_yaw = 0.1

trajectCompleted = False
tagDetectedHusky = False
tagDetectedJaco = False
counter = 0
counter_grasping = 0
graspingOn = False
taskCompleted = False

# Husky velocities for reference. vr -> linear. wr -> angular
vr = 0.3
wr = 0.0

# Controller parameters
k1 = 0.05
k2 = 1.0
k3 = 1.0

# Define speed as a Twist msg type
speed = Twist()
# Define pose_goal as a Pose msg type
pose_goal = Pose()



moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "Arm"
group = moveit_commander.MoveGroupCommander(group_name)
group_all = moveit_commander.MoveGroupCommander("All")


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
    pose_goal.position.x = 0.60 # 0.65 
    pose_goal.position.y = 0.00 # -0.05
    pose_goal.position.z = 0.530 # 0.66

    return pose_goal


def placePose():
    pose_goal.orientation.x = -0.016
    pose_goal.orientation.y = -0.029
    pose_goal.orientation.z = 0.712
    pose_goal.orientation.w = 0.700
    pose_goal.position.x = 0.305 
    pose_goal.position.y = -0.605
    pose_goal.position.z = 0.657

    return pose_goal


def pubPreGraspingPose():
    next_pose = preGraspingPose()
    group.set_pose_target(next_pose)
    plan2 = group.plan()   
    group.go(wait=True)
    group.stop()


def pubOpenFingers():
    group_variable_values = group_all.get_current_joint_values()
    group_variable_values[6] = 0.02
    group_variable_values[7] = 0.02
    group_variable_values[8] = 0.02

    group_all.set_joint_value_target(group_variable_values)
    plan2 = group_all.plan()
    group_all.go(wait=True)
    group_all.stop()


def pubCloseFingers():
    group_variable_values = group_all.get_current_joint_values()
    group_variable_values[6] = 0.53
    group_variable_values[7] = 0.53
    group_variable_values[8] = 0.53

    group_all.set_joint_value_target(group_variable_values)
    plan2 = group_all.plan()
    group_all.go(wait=True)
    group_all.stop()


def pubGraspingPose():
    pubOpenFingers()    
    graspingPose = preGraspingPose()

    graspingPose.position.y -= (jacoPositionY)
    graspingPose.position.z -= (jacoPositionZ - 0.11)
    group.set_pose_target(graspingPose)
    plan2 = group.plan()
    group.go(wait=True)
    group.stop()

    graspingPose.position.x += (jacoPositionX - 0.27)
    group.set_pose_target(graspingPose)
    plan2 = group.plan()
    group.go(wait=True)
    group.stop()

    rospy.sleep(1)
    pubCloseFingers()

    graspingPose.position.z += 0.10
    group.set_pose_target(graspingPose)
    plan2 = group.plan()
    group.go(wait=True)
    group.stop()

    graspingPose.position.x -= 0.20
    group.set_pose_target(graspingPose)
    plan2 = group.plan()
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()


def pubPlacePose():
    next_pose = placePose()
    group.set_pose_target(next_pose)
    plan2 = group.plan()   
    group.go(wait=True)
    group.stop()

    next_pose.position.x -= 0.20
    group.set_pose_target(next_pose)
    plan2 = group.plan()   
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

def pubHomePose():
    next_pose = homePose()
    group.set_pose_target(next_pose)
    plan2 = group.plan()
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()


# Function to calculate the errors
def getErrors(refYaw, refX, refY, huskyPositionX, huskyPositionY, huskyOrientationYaw):
    global e1
    global e2
    global e3
    e1 = (cos(-huskyOrientationYaw) * (refX - huskyPositionX)) + (sin(-huskyOrientationYaw) * (refY - (-huskyPositionY)))
    e2 = (-sin(-huskyOrientationYaw) * (refX - huskyPositionX)) + (cos(-huskyOrientationYaw) * (refY - (-huskyPositionY)))
    e3 = refYaw - (-huskyOrientationYaw)
    
    return e1, e2, e3


def getVelocities(vr, wr, k1, k2, k3):
    global trajectCompleted
    global error_tolerance
    errors = getErrors(refYaw, refX, refY, huskyPositionX, huskyPositionY, huskyOrientationYaw)
    v = (vr * cos(errors[2])) + (k1 * errors[0])
    w = wr + (vr * k2 * errors[1]) + (k3 * sin(errors[2]))
    trajectCompleted = False

    if ((abs(errors[0]) <= error_tolerance) and (abs(errors[1]) <= error_tolerance) and (abs(errors[2]) <= error_tolerance_yaw)):
        trajectCompleted = True
        v = 0.00000
        w = 0.00000

    return v, w


def getReference(msg):
    global huskyReferencePosition
    global huskyReferenceOrientation
    global jacoReferencePosition
    global tagDetectedJaco
    global tagDetectedHusky
    #global tagID
    global refX

    if (msg.detections[0].pose.header.frame_id == "camera_realsense_gazebo"):
        tagDetectedJaco = msg.detections[0]
        if (tagDetectedJaco.id[0] == 42):
             jacoReferencePosition = tagDetectedJaco.pose.pose.pose.position

    elif (msg.detections[0].pose.header.frame_id == "camera_depth_optical_frame"):
        tagDetectedHusky = msg.detections[0]
        if (tagDetectedHusky.id[0] != 42):
             huskyReferencePosition = tagDetectedHusky.pose.pose.pose.position
             huskyReferenceOrientation = tagDetectedHusky.pose.pose.pose.orientation

             if (tagDetectedHusky.id[0] == 0):
                 refX = 0.8
             elif (tagDetectedHusky.id[0] == 35):
                 refX = 0.6
             elif ((tagDetectedHusky.id[0] == 20) and (counter == 4)):
                 refX = 0.0
             else:
                 refX = 0.4

    else:
        huskyReferencePosition = Point()
        huskyReferenceOrientation = Point()


def turnLeft():
    global trajectCompleted
    tagNow = tagDetectedHusky.id[0]

    while (tagNow == tagDetectedHusky.id[0]):
        speed.linear.x = 0.0
        speed.angular.z = 0.3
        move.publish(speed)
    '''
    endTime = rospy.Time.now() + rospy.Duration(2)
    while (rospy.Time.now() < endTime):
        speed.linear.x = 0.0
        speed.angular.z = 0.3
        move.publish(speed)
    '''
    print("TagID: {}" .format(tagDetectedHusky.id[0]))
    print("TagNow: {}" .format(tagNow))

    trajectCompleted = False

    return trajectCompleted

rospy.init_node ("husky_jaco_navigation_pnp")
marker = rospy.Subscriber("tag_detections", AprilTagDetectionArray, getReference)
move = rospy.Publisher("husky_velocity_controller/cmd_vel", Twist, queue_size=1)
end_effector = rospy.Publisher("jaco/end_effector_pose", Pose, queue_size=1)
r = rospy.Rate(100)
jaco_pose = group.get_current_pose()


while not rospy.is_shutdown():

    # Transform the reference system from the tag to the reference system of the robots

    if (counter == 4):
        huskyPositionX = 0.0
        huskyPositionY = 0.0
        huskyOrientationYaw = huskyReferenceOrientation.z
        error_tolerance_yaw = 0.01
    
    else:
        huskyPositionX = huskyReferencePosition.z
        huskyPositionY =  huskyReferencePosition.x
        huskyOrientationYaw = huskyReferenceOrientation.z


    jacoPositionX = jacoReferencePosition.z
    jacoPositionY =  jacoReferencePosition.x
    jacoPositionZ =  jacoReferencePosition.y
    
    if (graspingOn == True):
        speed.linear.x = 0.000
        speed.angular.z = 0.000
    else:
        velocities = getVelocities(vr, wr, k1, k2, k3)
        speed.linear.x = velocities[0]
        speed.angular.z = velocities[1]

    move.publish(speed)
    end_effector.publish(jaco_pose.pose)

    if (taskCompleted and trajectCompleted):
        break

    if ((trajectCompleted == True) and (tagDetectedHusky.id[0] == 35) and (counter_grasping == 1)):
        pubGraspingPose()
        counter_grasping += 1
        graspingOn = False
        
    elif ((trajectCompleted == True) and (tagDetectedHusky.id[0] == 35) and (counter_grasping == 0)):
        #rospy.sleep(1)
        graspingOn = True
        pubPreGraspingPose()
        counter_grasping += 1

    elif ((trajectCompleted == True) and (tagDetectedHusky.id[0] == 0)):
        #rospy.sleep(1)
        pubPlacePose()
        pubOpenFingers()
        pubHomePose()
 

    while (trajectCompleted == True):
        if ((counter_grasping == 1) or (taskCompleted)):
            break
        else:
            if (tagDetectedHusky.id[0] == 0):
                 taskCompleted = True
            trajectCompleted = turnLeft()
            counter += 1

    print("Posicao X: {}" .format(huskyPositionX))
    print("Posicao Y: {}" .format(huskyPositionY))
    print("Orientacao Yaw: {}" .format(huskyOrientationYaw))
    print("Vel. Linear: {}" .format(speed.linear.x))
    print("Vel. Angular: {}" .format(speed.angular.z))
    print("Erro 1: {}" .format(e1))
    print("Erro 2: {}" .format(e2))
    print("Erro 3: {}" .format(e3))
    print("RefX: {}" .format(refX))
    print("RefY: {}" .format(refY))
    print("RefYaw: {}" .format(refYaw))
    print("Counter: {}" .format(counter))
    #print("Tag detectada: {}" .format(tagDetectedHusky.id[0]))

    r.sleep()
