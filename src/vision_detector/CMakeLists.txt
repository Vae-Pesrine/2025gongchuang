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
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)


add_message_files(
  FILES
  circle_position.msg
  circle_positions.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision_detector
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime
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

add_executable(test_zbar src/test_zbar.cpp)
add_dependencies(test_zbar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_zbar
  zbar
  ${catkin_LIBRARIES}
  ${OpenC_LIBRARIES}
)


# 1.添加可执行文件
file(GLOB_RECURSE source_detector 
  # ${CMAKE_SOURCE_DIR}/vision_detector/src/detector_node.cpp
	${CMAKE_SOURCE_DIR}/vision_detector/src/infer/*.cpp
	${CMAKE_SOURCE_DIR}/vision_detector/src/infer/*.cu
) 
add_executable(detector_node ${source_detector})
add_dependencies(detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(detector_node
  ${catkin_LIBRARIES}
  ${OpenC_LIBRARIES}
  ${CUDA_LIBRARIES}
  nvinfer nvinfer_plugin nvonnxparser nvparsers 
)

