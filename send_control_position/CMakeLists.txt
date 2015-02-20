cmake_minimum_required(VERSION 2.8.3)
project(send_control_position)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
		roscpp 
		std_msgs
		geometry_msgs)

###########
## Build ##
###########


include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(send_control_position_node src/control_data.cpp)
add_dependencies(send_control_position_node drone_control_msgs_generate_messages_cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(send_control_position_node ${catkin_LIBRARIES})


#############
## Install ##
#############



#############
## Testing ##
#############


