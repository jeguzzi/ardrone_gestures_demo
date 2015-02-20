cmake_minimum_required(VERSION 2.8.3)
project(drone_control_msgs)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder

FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

add_message_files(
   FILES
   ${messages_to_build}
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES 
   std_msgs 
   geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
		message_runtime
		std_msgs 
		geometry_msgs 
		
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(${catkin_INCLUDE_DIRS})


#############
## Install ##
#############




#############
## Testing ##
#############


