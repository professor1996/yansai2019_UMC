cmake_minimum_required(VERSION 2.8)
project(infrared)

find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

find_package(catkin REQUIRED COMPONENTS
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
#  LIBRARIES vision
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs 
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} src/main.cpp src/RMVideoCapture.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})	
