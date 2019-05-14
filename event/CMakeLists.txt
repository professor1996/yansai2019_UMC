
cmake_minimum_required(VERSION 3.5)

project(event)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

set(CeleX ./Samples)
set(CeleX_LIB CeleX)

include_directories(${CeleX}/include)
link_directories(${CeleX}/lib/Linux/x64)

if(CMAKE_VERSION VERSION_LESS "2.8.11")
    # Add OpenCV headers location to your include paths
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

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


find_package(catkin REQUIRED)
 
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/getFrameMat.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${CeleX_LIB})
