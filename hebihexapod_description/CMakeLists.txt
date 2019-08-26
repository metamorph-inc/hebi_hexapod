cmake_minimum_required(VERSION 2.8.3)
project(hebihexapod_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  actionlib
  std_msgs
  geometry_msgs
  sensor_msgs
  smach
  smach_ros
  hebiros
  hebiros_gazebo_plugin
  hebiros_utils
  urdf_parser_py
  trak_ik_python
  pykdl_utils
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   LegCmd.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hebihexapod_description
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
# joint_state_publisher robot_state_publisher roscpp rospy rviz sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
