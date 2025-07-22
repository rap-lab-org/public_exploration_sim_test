cmake_minimum_required(VERSION 3.0.2)
project(odom_manager)

## 编译为 C++17
add_compile_options(-std=c++17)

## 查找 catkin 包并声明依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
)

## 声明 catkin 包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES odom_manager
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs visualization_msgs tf2_ros
)

## 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 声明可执行文件
add_executable(odometry_manager_node src/odometry_manager_node.cpp src/vehicle.cpp src/agv.cpp src/uav.cpp src/odometry_manager.cpp)

## 链接库
target_link_libraries(odometry_manager_node
  ${catkin_LIBRARIES}
)
