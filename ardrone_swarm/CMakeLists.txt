cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(detector src/detectorNode.cpp)
set(GESTURE_SOURCE_FILES
    src/svm.cpp
)
set(GESTURE_HEADER_FILES
    src/svm.h
)
rosbuild_add_executable(detector src/detectorNode.cpp)
rosbuild_add_executable(detector2 src/detectorNode2.cpp)
rosbuild_add_executable(detector3 src/detectorNode2.cpp)
rosbuild_add_executable(detectorBig src/detectorNodeBig.cpp)
rosbuild_add_executable(calib src/calib.cpp)
rosbuild_add_executable(imageGrabber src/image_grabber.cpp)
#rosbuild_add_executable(gesture src/GestureSequenceMotionFinal_Alessandro.cpp ${GESTURE_SOURCE_FILES} ${GESTURE_HEADER_FILES})
target_link_libraries(detector ${PROJECT_SOURCE_DIR}/src/libcvblobs.a)
target_link_libraries(detector2 ${PROJECT_SOURCE_DIR}/src/libcvblobs.a)
target_link_libraries(detector3 ${PROJECT_SOURCE_DIR}/src/libcvblobs.a)
target_link_libraries(detectorBig ${PROJECT_SOURCE_DIR}/src/libcvblobs.a)
target_link_libraries(calib ${PROJECT_SOURCE_DIR}/src/libcvblobs.a)
target_link_libraries(imageGrabber ${PROJECT_SOURCE_DIR}/src/libcvblobs.a)
#target_link_libraries(detector ${PROJECT_SOURCE_DIR}/src/libcvblobs.a)

include_directories(/opt/ros/fuerte/include/opencv2)
#link_directories(src)

#rosbuild_add_executable(gesture2 src/GestureSequenceClassifierTest.cpp src/svm.cpp)
#rosbuild_add_executable(gesture2 src/GestureSimple.cpp) # src/svm.cpp)
SET(GCC_COVERAGE_COMPILE_FLAGS "-fpermissive")
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})
#target_link_libraries(gesture2 ${PROJECT_SOURCE_DIR}/src/libcvblobs.a)
