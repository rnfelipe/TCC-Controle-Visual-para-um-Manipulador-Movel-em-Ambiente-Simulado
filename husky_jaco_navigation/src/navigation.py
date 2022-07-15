#!/usr/bin/env python

from math import cos, sin
import rospy
from geometry_msgs.msg import Twist, Point
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry

referencePosition = Point()
referenceOrientation = Point()

positionOdom = Point()
orientationOdom = Point()

positionX = 0.0
positionY = 0.0
orientationYaw = 0.0

refX = 0.7
refY = 0.0
refYaw = 0.0

trajectCompleted = False
tagDetected = False
tagID = 1
counter = 0

# Husky velocities for reference. vr -> linear. wr -> angular
vr = 0.3
wr = 0.0

# Controller parameters
k1 = -0.001
k2 = 1.3
k3 = 1.0

speed = Twist()

# Function to calculate the errors
def getErrors(refYaw, refX, refY, positionX, positionY, orientationYaw):
    global e1
    global e2
    global e3
    e1 = (cos(orientationYaw) * (refX - positionX)) + (sin(orientationYaw) * (refY - positionY))
    e2 = (-sin(orientationYaw) * (refX - positionX)) + (cos(orientationYaw) * (refY - positionY))
    e3 = refYaw - orientationYaw
    
    return e1, e2, e3


def getVelocities(vr, wr, k1, k2, k3):
    global trajectCompleted
    errors = getErrors(refYaw, refX, refY, positionX, positionY, orientationYaw)
    v = (vr * cos(errors[2])) + (k1 * errors[0])
    w = wr + (vr * k2 * errors[1]) + (k3 * sin(errors[2]))
    trajectCompleted = False

    if ((abs(errors[0]) <= 0.1) and (abs(errors[1]) <= 0.1) and (abs(errors[2]) <= 0.1)):
        trajectCompleted = True
        v = 0.0
        w = 0.0

    return v, w


def getOdometryRef(msg):
    global positionOdom
    global orientationOdom

    positionOdom = msg.pose.pose.position
    orientationOdom = msg.pose.pose.orientation


def getReference(msg):
    global referencePosition
    global referenceOrientation
    global tagDetected
    global tagID

    tagDetected = msg.detections[0]
    tagID = tagDetected.id[0]

    if (tagDetected):
        referencePosition = tagDetected.pose.pose.pose.position
        referenceOrientation = tagDetected.pose.pose.pose.orientation
    else:
        referencePosition = Point()
        referenceOrientation = Point()

def turnLeft():
    global trajectCompleted
    tagNow = tagID

    while (tagNow == tagID):
        print("While TagID: {}" .format(tagID))
        print("While TagNow: {}" .format(tagNow))
        speed.linear.x = 0.0
        speed.angular.z = 0.4
        move.publish(speed)

    print("TagID: {}" .format(tagID))
    print("TagNow: {}" .format(tagNow))
    '''
    endTime = rospy.Time.now() + rospy.Duration(1.3)
    while (rospy.Time.now() < endTime):
        speed.linear.x = 0.0
        speed.angular.z = 0.5
        move.publish(speed)
        if ((endTime >= rospy.Time.now()) and (tagDetected == False)):
            endTime += rospy.Duration(0.1)
    '''
    trajectCompleted = False

    return trajectCompleted

rospy.init_node ("husky_jaco_navigation")

odom = rospy.Subscriber("odometry/filtered", Odometry, getOdometryRef)
marker = rospy.Subscriber("tag_detections", AprilTagDetectionArray, getReference)
move = rospy.Publisher("husky_velocity_controller/cmd_vel", Twist, queue_size=1)

r = rospy.Rate(100)


while not rospy.is_shutdown():

    positionX = referencePosition.z # positionOdom.x
    positionY =  referencePosition.x # positionOdom.y
    orientationYaw = referenceOrientation.z # orientationOdom.z
    
    velocities = getVelocities(vr, wr, k1, k2, k3)
    speed.linear.x = velocities[0]
    speed.angular.z = velocities[1]

    move.publish(speed)

    while ((trajectCompleted == True) and (tagID != 0)): #and (e1 != 0.0)):
        trajectCompleted = turnLeft()
        counter += 1

    print("Posicao X: {}" .format(positionX))
    print("Posicao Y: {}" .format(positionY))
    print("Orientacao Yaw: {}" .format(orientationYaw))
    print("Vel. Linear: {}" .format(speed.linear.x))
    print("Vel. Angular: {}" .format(speed.angular.z))
    print("Erro 1: {}" .format(e1))
    print("Erro 2: {}" .format(e2))
    print("Erro 3: {}" .format(e3))
    print("RefX: {}" .format(refX))
    print("RefY: {}" .format(refY))
    print("RefYaw: {}" .format(refYaw))
    print("Counter: {}" .format(counter))
    print("Tag detectada: {}" .format(tagID))

    r.sleep()
