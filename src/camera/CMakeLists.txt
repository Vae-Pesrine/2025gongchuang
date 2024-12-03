cmake_minimum_required(VERSION 3.0.2)
project(camera)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ~/little_car/src/camera/include
)


add_executable(use_camera src/use_camera.cpp)
add_dependencies(use_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(use_camera
  ${catkin_LIBRARIES}
  ${OpenC_LIBRARIES}
)


add_executable(hsv src/hsv.cpp)
add_dependencies(hsv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hsv
  ${catkin_LIBRARIES}
  ${OpenC_LIBRARIES}
)