cmake_minimum_required(VERSION 3.0.2)
project(vision_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision_detector
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(get_image_node src/get_image_node.cpp)
add_dependencies(get_image_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_image_node
  ${catkin_LIBRARIES}
  ${OpenC_LIBRARIES}
)