cmake_minimum_required(VERSION 2.8.3)
project(sim_optitrack)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros
	geometry_msgs
	message_generation
	roscpp
	rospy
	std_msgs
	sim_optitrack_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

#######################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
		roscpp 
		std_msgs
		message_runtime 
		geometry_msgs
		sim_optitrack_msgs
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(SimOptitrack_node src/optitrack_simulator.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(SimOptitrack_node  ${catkin_EXPORTED_TARGETS} )

## Specify libraries to link a library or executable target against
target_link_libraries(SimOptitrack_node ${catkin_LIBRARIES})


