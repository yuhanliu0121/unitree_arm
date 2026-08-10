cmake_minimum_required(VERSION 3.8)
project(d1_ros2_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

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

set(D1_SDK_MSG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../D1_SDK/src/msg")
if(NOT EXISTS "${D1_SDK_MSG_DIR}/ArmString_.hpp")
  message(FATAL_ERROR "Expected D1 SDK messages at ${D1_SDK_MSG_DIR}")
endif()

add_library(
  ${PROJECT_NAME}
  SHARED
  src/d1_system_hardware.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PRIVATE D1_ROS2_CONTROL_BUILDING_DLL)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_executable(
  d1_dds_gateway
  src/d1_dds_gateway.cpp
  ${D1_SDK_MSG_DIR}/ArmString_.cpp
  ${D1_SDK_MSG_DIR}/PubServoInfo_.cpp
)
target_compile_features(d1_dds_gateway PUBLIC cxx_std_17)
target_include_directories(
  d1_dds_gateway
  PRIVATE
    include
    ${D1_SDK_MSG_DIR}
    ${UNITREE_SDK2_INCLUDE_DIR}
    ${CYCLONEDDS_CXX_INCLUDE_DIR}
)
target_link_libraries(
  d1_dds_gateway
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
  d1_dds_gateway
  PRIVATE "-Wl,--disable-new-dtags,-rpath,/usr/local/lib"
)

pluginlib_export_plugin_description_file(
  hardware_interface
  d1_ros2_control_plugin.xml
)

install(
  DIRECTORY include/
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
  TARGETS ${PROJECT_NAME} d1_dds_gateway
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
