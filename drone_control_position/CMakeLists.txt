cmake_minimum_required(VERSION 2.8.3)
project(drone_control_position)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  roscpp
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(	CATKIN_DEPENDS
		roscpp 
		std_msgs
		geometry_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})


## Declare a cpp executable
add_executable(sim_control_drone_position_node src/sim_control_node.cpp)
add_executable(control_drone_position_node src/control_node.cpp)
#add_executable(sim_following_leader_node src/sim_following_leader.cpp)
#add_executable(following_leader_node src/following_leader.cpp)

add_dependencies(sim_control_drone_position_node drone_control_msgs_generate_messages_cpp)
add_dependencies(control_drone_position_node drone_control_msgs_generate_messages_cpp)
#add_dependencies(sim_following_leader_node drone_control_msgs_generate_messages_cpp)
#add_dependencies(following_leader_node drone_control_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(sim_control_drone_position_node ${catkin_LIBRARIES})
target_link_libraries(control_drone_position_node ${catkin_LIBRARIES})
#target_link_libraries(sim_following_leader_node ${catkin_LIBRARIES})
#target_link_libraries(following_leader_node ${catkin_LIBRARIES})
