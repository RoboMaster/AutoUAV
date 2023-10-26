#  Copyright © 2023 ROBOMASTER All Rights Reserved.
#  You may use, distribute and modify this code under the
#  terms of the MIT license, which unfortunately won't be
#  written for another century.

cmake_minimum_required(VERSION 3.0.2)
project(d430_slam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Pangolin REQUIRED PATHS /home/nvidia/Softwares/Pangolin/build)
find_package(Eigen3 REQUIRED)
find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    image_transport
)
find_package(OpenCV 4.2.0 REQUIRED EXACT)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy image_transport 
#  DEPENDS system_lib
)

set(ORBSLAM3_PATH /home/nvidia/Softwares/ORB_SLAM3)
find_package(realsense2 REQUIRED)

include_directories(
include
${ORBSLAM3_PATH}
${ORBSLAM3_PATH}/Thirdparty/Sophus/
${ORBSLAM3_PATH}/Thirdparty/
${ORBSLAM3_PATH}/include/
${ORBSLAM3_PATH}/include/CameraModels/
${catkin_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
${REALSENSE2_INCLUDE_DIR}
)

## Declare a C++ library
add_library(d430_camera
  src/node/d430_camera.cpp
)

target_link_libraries(d430_camera
  ${catkin_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${realsense2_LIBRARY}
  ${OpenCV_LIBS}
)

add_library(orbslam
  src/node/orbslam.cpp
)

target_link_libraries(orbslam
  ${realsense2_LIBRARY}
  ${catkin_LIBRARIES}
  d430_camera
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${ORBSLAM3_PATH}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORBSLAM3_PATH}/Thirdparty/g2o/lib/libg2o.so
  ${ORBSLAM3_PATH}/lib/libORB_SLAM3.so
  -lboost_system
)

add_executable(${PROJECT_NAME}_node src/node/d430_slam_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  d430_camera
  orbslam
  ${realsense2_LIBRARY}
)

