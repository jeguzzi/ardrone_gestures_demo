cmake_minimum_required(VERSION 2.8.3)
project(direction_beta)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  roscpp
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

catkin_package(	CATKIN_DEPENDS
		roscpp 
		std_msgs
		geometry_msgs
		message_runtime
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})


## Declare a cpp executable
#add_executable(sim_control_direction_node src/sim_control_node.cpp)
add_executable(drone_control_direction src/control_node.cpp)

#add_dependencies(sim_control_direction_node direction_beta_generate_messages_cpp)
add_dependencies(drone_control_direction direction_beta_generate_messages_cpp)


## Specify libraries to link a library or executable target against
#target_link_libraries(sim_control_direction_node ${catkin_LIBRARIES})
target_link_libraries(drone_control_direction ${catkin_LIBRARIES})


