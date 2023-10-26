cmake_minimum_required(VERSION 3.0.2)
project(apm_d430_bridge)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  tf
  mavros
  mavros_extras
  mavros_msgs
  mavlink
)

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
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

set(APM_D430_BRIDGE_CPP_FILES   "src/nodes/apm_d430_bridge.cpp"
                              "src/nodes/apm_d430_bridge_node.cpp"
)

add_library(apm_d430_bridge     "${APM_D430_BRIDGE_CPP_FILES}")

add_dependencies(apm_d430_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(apm_d430_bridge_node src/nodes/apm_d430_bridge_node.cpp)

target_link_libraries(apm_d430_bridge_node
  apm_d430_bridge
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

