cmake_minimum_required(VERSION 2.8.3)
project(blender_rgbd_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    roslib
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
)

catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

catkin_install_python(
  PROGRAMS scripts/depth_rgb_pub.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)