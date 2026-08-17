cmake_minimum_required(VERSION 3.8)
project(robot_collision_observer)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robot_collision_observer_node src/collision_observer_node.cpp)
target_compile_features(robot_collision_observer_node PUBLIC cxx_std_17)
ament_target_dependencies(
  robot_collision_observer_node
  gazebo_msgs
  rclcpp
  std_msgs
)

install(TARGETS robot_collision_observer_node DESTINATION lib/${PROJECT_NAME})

ament_package()
