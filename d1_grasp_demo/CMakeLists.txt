cmake_minimum_required(VERSION 3.8)
project(d1_grasp_demo)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(pick_cube src/pick_cube.cpp)
target_compile_features(pick_cube PRIVATE cxx_std_17)
ament_target_dependencies(
  pick_cube
  control_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  shape_msgs
)

install(TARGETS pick_cube DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
