cmake_minimum_required(VERSION 3.5)
project(turtle_chase_package)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(include)

add_executable(turtle_chase src/turtle_chase.cpp)
ament_target_dependencies(turtle_chase
rclcpp
geometry_msgs
turtlesim
std_msgs)

install(TARGETS
  turtle_chase
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
