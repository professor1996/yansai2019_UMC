########### 版本要求
cmake_minimum_required(VERSION 2.8.3)

########### 项目名称
project(velodyne)

########### 添加系统库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(CATKIN_DEPENDS
)

########### 添加头文件
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

########### 编译可执行文件
#add_executable(xxx src/xxx.cpp)
#target_link_libraries(xxx ${catkin_LIBRARIES})
