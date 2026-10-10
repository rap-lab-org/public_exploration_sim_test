cmake_minimum_required(VERSION 3.8)
project(rch_benchmark)

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
find_package(explorer REQUIRED)

add_executable(rch_benchmark src/rch_benchmark.cpp)
target_link_libraries(rch_benchmark explorer::rch)

install(TARGETS rch_benchmark DESTINATION lib/${PROJECT_NAME})

ament_package()
