cmake_minimum_required(VERSION 3.5)
project(autocar_map)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(bof src/bof.cpp)
ament_target_dependencies(bof rclcpp std_msgs nav_msgs tf2 tf2_ros geometry_msgs sensor_msgs)

install(TARGETS
  bof
  DESTINATION lib/${PROJECT_NAME})

ament_package()