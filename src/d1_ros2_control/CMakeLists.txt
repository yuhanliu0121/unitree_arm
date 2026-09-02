cmake_minimum_required(VERSION 3.8)
project(d1_ros2_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteJointSegment.action"
)

find_path(
  UNITREE_SDK2_INCLUDE_DIR
  unitree/robot/channel/channel_subscriber.hpp
  PATHS /usr/local/include /usr/include
)
find_path(
  CYCLONEDDS_CXX_INCLUDE_DIR
  dds/dds.hpp
  PATHS /usr/local/include/ddscxx /usr/include/ddscxx
)
find_library(
  UNITREE_SDK2_LIBRARY
  NAMES unitree_sdk2
  PATHS /usr/local/lib /usr/lib
)
find_library(
  CYCLONEDDS_C_LIBRARY
  NAMES ddsc
  PATHS /usr/local/lib /usr/lib
)
find_library(
  CYCLONEDDS_CXX_LIBRARY
  NAMES ddscxx
  PATHS /usr/local/lib /usr/lib
)

foreach(required_value
    UNITREE_SDK2_INCLUDE_DIR
    CYCLONEDDS_CXX_INCLUDE_DIR
    UNITREE_SDK2_LIBRARY
    CYCLONEDDS_C_LIBRARY
    CYCLONEDDS_CXX_LIBRARY)
  if(NOT ${required_value})
    message(
      FATAL_ERROR
      "Missing ${required_value}. Install unitree_sdk2 and Cyclone DDS/C++."
    )
  endif()
endforeach()

set(D1_SDK_MSG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/D1_SDK/src/msg")
set(D1_CONTROL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree-d1-control")
if(NOT EXISTS "${D1_SDK_MSG_DIR}/ArmString_.hpp")
  message(FATAL_ERROR "Expected D1 SDK messages at ${D1_SDK_MSG_DIR}")
endif()
if(NOT EXISTS "${D1_CONTROL_DIR}/host/src/d1_control_gateway.cpp")
  message(FATAL_ERROR "Expected unitree-d1-control at ${D1_CONTROL_DIR}")
endif()

set(D1_HARDWARE_TARGET ${PROJECT_NAME}_hardware)
add_library(
  ${D1_HARDWARE_TARGET}
  SHARED
  src/d1_system_hardware.cpp
)

set_target_properties(${D1_HARDWARE_TARGET} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_compile_features(${D1_HARDWARE_TARGET} PUBLIC cxx_std_17)
target_compile_definitions(${D1_HARDWARE_TARGET} PRIVATE D1_ROS2_CONTROL_BUILDING_DLL)
target_include_directories(
  ${D1_HARDWARE_TARGET}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${D1_CONTROL_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${D1_HARDWARE_TARGET}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_executable(
  d1_control_gateway
  ${D1_CONTROL_DIR}/host/src/d1_control_gateway.cpp
  ${D1_SDK_MSG_DIR}/ArmString_.cpp
  ${D1_SDK_MSG_DIR}/PubServoInfo_.cpp
)
target_compile_features(d1_control_gateway PUBLIC cxx_std_17)
target_include_directories(
  d1_control_gateway
  PRIVATE
    include
    ${D1_CONTROL_DIR}/include
    ${D1_SDK_MSG_DIR}
    ${UNITREE_SDK2_INCLUDE_DIR}
    ${CYCLONEDDS_CXX_INCLUDE_DIR}
)
target_link_libraries(
  d1_control_gateway
  ${UNITREE_SDK2_LIBRARY}
  ${CYCLONEDDS_C_LIBRARY}
  ${CYCLONEDDS_CXX_LIBRARY}
  rt
  pthread
)
# ROS 2 Humble also ships libddsc.so.0. The Unitree SDK and its generated C++
# types were built against the /usr/local pair; mixing ROS's C library with
# /usr/local's C++ library corrupts topic metadata. DT_RPATH intentionally
# takes precedence over an inherited ROS LD_LIBRARY_PATH for this executable.
target_link_options(
  d1_control_gateway
  PRIVATE "-Wl,--disable-new-dtags,-rpath,/usr/local/lib"
)

add_executable(d1_joint_segment_controller src/d1_joint_segment_controller.cpp)
target_compile_features(d1_joint_segment_controller PUBLIC cxx_std_17)
target_include_directories(
  d1_joint_segment_controller PRIVATE include ${D1_CONTROL_DIR}/include)
ament_target_dependencies(
  d1_joint_segment_controller
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
)
rosidl_get_typesupport_target(d1_controller_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(d1_joint_segment_controller "${d1_controller_typesupport_target}")

pluginlib_export_plugin_description_file(
  hardware_interface
  d1_ros2_control_plugin.xml
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY ${D1_CONTROL_DIR}/include/d1_control
  DESTINATION include
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES README.md d1_ros2_control_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS ${D1_HARDWARE_TARGET}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  TARGETS d1_control_gateway d1_joint_segment_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/d1_gripper_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface pluginlib rclcpp rclcpp_lifecycle rosidl_default_runtime)
ament_package()
