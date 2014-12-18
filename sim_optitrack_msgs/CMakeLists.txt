cmake_minimum_required(VERSION 2.8.3)
project(sim_optitrack_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

# search for all msg files
FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

# notify catkin to look at the previously found msg files
add_message_files(
   FILES
   ${messages_to_build}
 )

# build the header files from the msg files, and notify catkin about the dependencies
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

# export the dependencis of this package for who ever depends on us
catkin_package(
CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

