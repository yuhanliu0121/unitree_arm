cmake_minimum_required(VERSION 3.16)
project(d1_control VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(D1_CONTROL_BUILD_HOST "Build the host UDP-to-D1 DDS gateway" ON)
option(D1_CONTROL_BUILD_ONBOARD "Build the D1-computer serial dispatcher" OFF)
option(D1_CONTROL_BUILD_TESTS "Build dependency-free protocol tests" ON)
set(
  D1_OFFICIAL_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../D1_SDK"
  CACHE PATH "Path to the official D1 SDK source tree"
)
set(D1_OFFICIAL_MSG_DIR "${D1_OFFICIAL_SDK_ROOT}/src/msg")

if(D1_CONTROL_BUILD_HOST OR D1_CONTROL_BUILD_ONBOARD)
  foreach(message_file ArmString_ PubServoInfo_ SetServoDumping_)
    if(NOT EXISTS "${D1_OFFICIAL_MSG_DIR}/${message_file}.hpp")
      message(FATAL_ERROR "Missing official D1 message ${message_file}.hpp")
    endif()
  endforeach()

  find_path(
    UNITREE_SDK2_INCLUDE_DIR unitree/robot/channel/channel_publisher.hpp
    PATHS /usr/local/include /usr/include REQUIRED
  )
  find_path(
    CYCLONEDDS_CXX_INCLUDE_DIR dds/dds.hpp
    PATHS /usr/local/include/ddscxx /usr/include/ddscxx REQUIRED
  )
  find_library(
    UNITREE_SDK2_LIBRARY NAMES unitree_sdk2
    PATHS /usr/local/lib /usr/lib REQUIRED
  )
  find_library(
    CYCLONEDDS_C_LIBRARY NAMES ddsc
    PATHS /usr/local/lib /usr/lib REQUIRED
  )
  find_library(
    CYCLONEDDS_CXX_LIBRARY NAMES ddscxx
    PATHS /usr/local/lib /usr/lib REQUIRED
  )
endif()

function(d1_control_configure_unitree_target target)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(
    ${target}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${D1_OFFICIAL_SDK_ROOT}/src
      ${D1_OFFICIAL_MSG_DIR}
      ${UNITREE_SDK2_INCLUDE_DIR}
      ${CYCLONEDDS_CXX_INCLUDE_DIR}
  )
  target_link_libraries(
    ${target}
    PRIVATE
      ${UNITREE_SDK2_LIBRARY}
      ${CYCLONEDDS_C_LIBRARY}
      ${CYCLONEDDS_CXX_LIBRARY}
      rt
      pthread
  )
  # ROS 2 Humble may prepend an ABI-incompatible Cyclone DDS C library.
  target_link_options(
    ${target} PRIVATE "-Wl,--disable-new-dtags,-rpath,/usr/local/lib"
  )
endfunction()

if(D1_CONTROL_BUILD_HOST)
  add_executable(
    d1_control_gateway
    host/src/d1_control_gateway.cpp
    ${D1_OFFICIAL_MSG_DIR}/ArmString_.cpp
    ${D1_OFFICIAL_MSG_DIR}/PubServoInfo_.cpp
  )
  d1_control_configure_unitree_target(d1_control_gateway)
  foreach(
      maintenance_tool
      d1_all_joints_unload
      d1_move_stowed
      d1_move_zero
      all_joints_unload
      arm_stowed_control
      arm_zero_control)
    add_executable(
      ${maintenance_tool}
      host/src/d1_maintenance_tool.cpp
      ${D1_OFFICIAL_MSG_DIR}/ArmString_.cpp
      ${D1_OFFICIAL_MSG_DIR}/PubServoInfo_.cpp
      ${D1_OFFICIAL_MSG_DIR}/SetServoDumping_.cpp
    )
    d1_control_configure_unitree_target(${maintenance_tool})
  endforeach()
  target_compile_definitions(d1_all_joints_unload PRIVATE D1_MAINTENANCE_UNLOAD)
  target_compile_definitions(d1_move_stowed PRIVATE D1_MAINTENANCE_STOWED)
  target_compile_definitions(d1_move_zero PRIVATE D1_MAINTENANCE_ZERO)
  target_compile_definitions(all_joints_unload PRIVATE D1_MAINTENANCE_UNLOAD)
  target_compile_definitions(arm_stowed_control PRIVATE D1_MAINTENANCE_STOWED)
  target_compile_definitions(arm_zero_control PRIVATE D1_MAINTENANCE_ZERO)

  foreach(
      joint_tool
      joint_angle_control
      multiple_joint_angle_control
      joint_delta_control
      joint_enable_control)
    add_executable(
      ${joint_tool}
      host/src/d1_joint_command_tool.cpp
      ${D1_OFFICIAL_MSG_DIR}/ArmString_.cpp
      ${D1_OFFICIAL_MSG_DIR}/PubServoInfo_.cpp
      ${D1_OFFICIAL_MSG_DIR}/SetServoDumping_.cpp
    )
    d1_control_configure_unitree_target(${joint_tool})
  endforeach()
  target_compile_definitions(joint_angle_control PRIVATE D1_TOOL_SINGLE)
  target_compile_definitions(multiple_joint_angle_control PRIVATE D1_TOOL_MULTIPLE)
  target_compile_definitions(joint_delta_control PRIVATE D1_TOOL_DELTA)
  target_compile_definitions(joint_enable_control PRIVATE D1_TOOL_ENABLE)

  foreach(feedback_tool get_arm_joint_angle d1_interface_probe)
    add_executable(
      ${feedback_tool}
      host/src/d1_feedback_monitor.cpp
      ${D1_OFFICIAL_MSG_DIR}/PubServoInfo_.cpp
    )
    d1_control_configure_unitree_target(${feedback_tool})
  endforeach()
  target_compile_definitions(d1_interface_probe PRIVATE D1_INTERFACE_PROBE)
  install(
    TARGETS
      d1_control_gateway
      d1_all_joints_unload
      d1_move_stowed
      d1_move_zero
      all_joints_unload
      arm_stowed_control
      arm_zero_control
      joint_angle_control
      multiple_joint_angle_control
      joint_delta_control
      joint_enable_control
      get_arm_joint_angle
      d1_interface_probe
    RUNTIME DESTINATION bin
  )
endif()

if(D1_CONTROL_BUILD_ONBOARD)
  find_path(
    FASHIONSTAR_INCLUDE_DIR FashionStar/UServo/FashionStar_UartServo.h
    PATHS /usr/local/include REQUIRED
  )
  find_library(FASHIONSTAR_LIBRARY NAMES fsuartservo PATHS /usr/local/lib REQUIRED)
  find_library(CSERIALPORT_LIBRARY NAMES cserialport PATHS /usr/local/lib REQUIRED)
  add_executable(
    d1_control_node
    onboard/src/d1_control_node.cpp
    ${D1_OFFICIAL_MSG_DIR}/ArmString_.cpp
    ${D1_OFFICIAL_MSG_DIR}/PubServoInfo_.cpp
    ${D1_OFFICIAL_MSG_DIR}/SetServoDumping_.cpp
  )
  d1_control_configure_unitree_target(d1_control_node)
  target_include_directories(d1_control_node PRIVATE ${FASHIONSTAR_INCLUDE_DIR})
  target_link_libraries(
    d1_control_node PRIVATE ${FASHIONSTAR_LIBRARY} ${CSERIALPORT_LIBRARY}
  )
  add_executable(
    d1_native_pose_control
    onboard/src/d1_native_pose_control.cpp
  )
  target_compile_options(d1_native_pose_control PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(
    d1_native_pose_control
    PRIVATE ${FASHIONSTAR_INCLUDE_DIR}
  )
  target_link_libraries(
    d1_native_pose_control
    PRIVATE ${FASHIONSTAR_LIBRARY} ${CSERIALPORT_LIBRARY} pthread
  )
  target_link_options(
    d1_native_pose_control PRIVATE "-Wl,--disable-new-dtags,-rpath,/usr/local/lib"
  )
  install(
    TARGETS d1_control_node d1_native_pose_control
    RUNTIME DESTINATION bin
  )
endif()

if(D1_CONTROL_BUILD_TESTS)
  include(CTest)
  add_executable(local_protocol_test tests/local_protocol_test.cpp)
  target_include_directories(local_protocol_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_options(local_protocol_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME local_protocol_test COMMAND local_protocol_test)
endif()

install(DIRECTORY include/d1_control DESTINATION include)
install(
  FILES README.md LICENSE NOTICE.md CONTRIBUTING.md SECURITY.md CHANGELOG.md
  DESTINATION share/d1_control
)
install(
  DIRECTORY docs/
  DESTINATION share/d1_control/docs
)
