cmake_minimum_required(VERSION 3.8)
project(explorer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# RCH solver, ROS-independent. Linked by src/rch_benchmark, keep the target name.
add_library(rch src/rch.cpp)
target_include_directories(rch PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(explorer_node src/explorer_node.cpp)
target_link_libraries(explorer_node rch)
ament_target_dependencies(explorer_node rclcpp nav_msgs geometry_msgs sensor_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rch EXPORT export_rch
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS explorer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_rch HAS_LIBRARY_TARGET)
ament_package()
