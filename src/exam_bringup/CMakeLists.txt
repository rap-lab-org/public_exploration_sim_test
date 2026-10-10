cmake_minimum_required(VERSION 3.8)
project(exam_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(evaluator src/evaluator.cpp)
ament_target_dependencies(evaluator rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs)

add_executable(scan_merger src/scan_merger.cpp)
ament_target_dependencies(scan_merger rclcpp sensor_msgs)

install(TARGETS evaluator scan_merger DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
