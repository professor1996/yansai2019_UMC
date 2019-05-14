cmake_minimum_required(VERSION 2.8.3)
project(control)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
 
nav_msgs
sensor_msgs
tf
tf2

  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial
CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs   nav_msgs tf sensor_msgs tf2 

#  DEPENDS system_lib
)



include_directories(
 include
  ${catkin_INCLUDE_DIRS} 
)


add_executable(control_node src/MainControl.cpp)

target_link_libraries(control_node
${OpenCV_LIBS} 
${catkin_LIBRARIES}
)
  
