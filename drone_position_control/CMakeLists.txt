cmake_minimum_required(VERSION 2.8.3)
project(drone_position_control)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

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

add_dependencies(sim_control_drone_position_node gesture_msgs_generate_messages_cpp)
add_dependencies(control_drone_position_node gesture_msgs_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(sim_control_drone_position_node ${catkin_LIBRARIES})
target_link_libraries(control_drone_position_node ${catkin_LIBRARIES})


