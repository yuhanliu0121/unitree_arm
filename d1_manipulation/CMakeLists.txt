cmake_minimum_required(VERSION 3.8)
project(d1_manipulation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ObserveTarget.action"
  "action/PickObject.action"
  "action/DropObject.action"
  "srv/DetectTarget.srv"
  "srv/EstimateCube.srv"
  "srv/VerifyHeldObject.srv"
  DEPENDENCIES geometry_msgs
)

ament_python_install_package(d1_perception_adapter
  PACKAGE_DIR python/d1_perception_adapter
)

add_library(observation_geometry src/observation_geometry.cpp)
target_compile_features(observation_geometry PUBLIC cxx_std_17)
target_include_directories(observation_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(observation_geometry Eigen3::Eigen)

add_executable(observe_target_server src/observe_target_server.cpp)
target_compile_features(observe_target_server PRIVATE cxx_std_17)
target_link_libraries(observe_target_server observation_geometry Eigen3::Eigen)
ament_target_dependencies(observe_target_server
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  sensor_msgs
  shape_msgs
  std_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_executable(pick_object_server src/pick_object_server.cpp)
target_compile_features(pick_object_server PRIVATE cxx_std_17)
target_link_libraries(pick_object_server Eigen3::Eigen)
ament_target_dependencies(pick_object_server
  control_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  shape_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  visualization_msgs
)
add_executable(drop_object_server src/drop_object_server.cpp)
target_compile_features(drop_object_server PRIVATE cxx_std_17)
target_link_libraries(drop_object_server Eigen3::Eigen)
ament_target_dependencies(drop_object_server
  control_msgs geometry_msgs moveit_core moveit_msgs
  moveit_ros_planning_interface rclcpp rclcpp_action shape_msgs
  tf2_eigen tf2_ros trajectory_msgs visualization_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(observe_target_server "${cpp_typesupport_target}")
target_link_libraries(pick_object_server "${cpp_typesupport_target}")
target_link_libraries(drop_object_server "${cpp_typesupport_target}")

install(TARGETS observation_geometry observe_target_server pick_object_server drop_object_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/detect_target_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME detect_target_server
)
install(PROGRAMS scripts/observe_seed_cube.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME observe_seed_cube
)
install(PROGRAMS scripts/pick_seed_cube.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_seed_cube
)
install(PROGRAMS scripts/drop_seed_bin.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME drop_seed_bin
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_observation_geometry test/test_observation_geometry.cpp)
  target_link_libraries(test_observation_geometry observation_geometry Eigen3::Eigen)
  ament_add_pytest_test(test_perception_adapter test/test_perception_adapter.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(observation_geometry)
ament_package()
