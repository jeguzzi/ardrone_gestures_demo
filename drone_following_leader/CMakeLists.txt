cmake_minimum_required(VERSION 2.8.3)
project(drone_following_leader)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################



###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(following_leader_node src/following_leader.cpp)
add_dependencies(following_leader_node drone_control_msgs_generate_messages_cpp)
target_link_libraries(following_leader_node ${catkin_LIBRARIES})


#############
## Install ##
#############


#############
## Testing ##
#############


