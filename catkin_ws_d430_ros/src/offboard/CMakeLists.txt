#  Copyright © 2023 ROBOMASTER All Rights Reserved.
#  You may use, distribute and modify this code under the
#  terms of the MIT license, which unfortunately won't be
#  written for another century.

cmake_minimum_required(VERSION 3.0.2)
project(offboard)

add_definitions(-std=c++14)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
#   dynamic_reconfigure
  tf
#   pcl_ros
  mavros
  mavros_extras
  mavros_msgs
  mavlink
)
# find_package(PCL 1.7 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs mav_msgs sensor_msgs message_runtime tf
#  DEPENDS system_lib
)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
#   ${YAML_CPP_INCLUDE_DIR}
)

add_library(offboard_rover    src/node/offboard_rover.cpp)


add_dependencies(
  offboard_rover ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${EIGEN3_LIBS}
)

add_library(offboard_postest     src/node/offboard_postest.cpp)


add_dependencies(
  offboard_postest ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${EIGEN3_LIBS}
)

add_library(offboard_veltest    src/node/offboard_veltest.cpp  utils/pid.cpp)


add_dependencies(
  offboard_veltest ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${EIGEN3_LIBS}
)

add_executable(offboard_test_node src/node/offboard_test_node.cpp)

target_link_libraries(offboard_test_node
  offboard_rover
  offboard_postest
  offboard_veltest
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

