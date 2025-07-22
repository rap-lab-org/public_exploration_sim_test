cmake_minimum_required(VERSION 3.0.2)
project(pcd_publisher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 查找所需组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  roslib
)

## 声明 catkin 包
catkin_package()

## 包含头文件目录
include_directories(
  include  # 添加 include 目录
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## 添加可执行文件
add_executable(${PROJECT_NAME}_node src/pcd_publisher_node.cpp src/pcd_publisher.cpp)

## 链接库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

