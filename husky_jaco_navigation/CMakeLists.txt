cmake_minimum_required(VERSION 2.8.3)
project(husky_jaco_navigation)
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  apriltag_ros
)

catkin_package(
CATKIN_DEPENDS rospy geometry_msgs apriltag_ros
)

install(PROGRAMS
  src/navigation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
