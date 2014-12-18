cmake_minimum_required(VERSION 2.8.3)
project(drone_gestures_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		std_msgs
		genmsg
		cv_bridge
		message_runtime
		image_transport
		gesture_msgs
)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
		roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

include_directories(/opt/ros/indigo/lib/sound_play)

## Declare a cpp executable

add_executable(detectorBig src/detectorNodeBig.cpp)
add_executable(vision src/vision.cpp)
#add_executable(cosa src/cosa.cpp)

#target_link_libraries(cosa ${catkin_LIBRARIES})
target_link_libraries(detectorBig  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/cvBlobsLib/lib/libblob.a ${OpenCV_LIBRARIES} pthread)
target_link_libraries(vision ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/cvBlobsLib/lib/libblob.a ${OpenCV_LIBRARIES} pthread)

#############
## Install ##
#############

 install(PROGRAMS	scripts/controller.py  
			scripts/controller_main.py 
		 	scripts/drone_info.py  
			scripts/keyboard_controller.py
			scripts/drone_video_display2.py
		 	scripts/tag.py
			scripts/utils.py
			scripts/sounds.py
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

#############
## Testing ##
#############

SET(GCC_COVERAGE_COMPILE_FLAGS "-fpermissive")
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})
