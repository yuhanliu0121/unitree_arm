cmake_minimum_required(VERSION 3.8)
project(d1_workspace_analysis)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(d1_manipulation REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(grasp_workspace_scan src/grasp_workspace_scan.cpp)
target_compile_features(grasp_workspace_scan PRIVATE cxx_std_17)
target_link_libraries(grasp_workspace_scan
  Eigen3::Eigen
  d1_manipulation::observation_geometry
)
ament_target_dependencies(grasp_workspace_scan
  d1_manipulation
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rclcpp
  shape_msgs
)

install(TARGETS grasp_workspace_scan
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
