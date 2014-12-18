cmake_minimum_required(VERSION 2.8.3)
project(simulation_worlds)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rosbag
  roscpp
  rospy
  std_msgs
  tf
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS roscpp gazebo_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)


## Install ##
#############


#############
## Testing ##
#############


