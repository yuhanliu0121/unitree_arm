cmake_minimum_required(VERSION 3.8)
project(d1_constrained_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/${PROJECT_NAME}
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

add_executable(
  d1_joint_state_bridge
  src/d1_joint_state_bridge.cpp
  src/msg/PubServoInfo_.cpp
)

target_compile_features(d1_joint_state_bridge PRIVATE cxx_std_17)

target_include_directories(
  d1_joint_state_bridge
  PRIVATE
    src
    ${UNITREE_SDK2_INCLUDE_DIR}
    ${CYCLONEDDS_CXX_INCLUDE_DIR}
)

target_link_libraries(
  d1_joint_state_bridge
  ${UNITREE_SDK2_LIBRARY}
  ${CYCLONEDDS_C_LIBRARY}
  ${CYCLONEDDS_CXX_LIBRARY}
  rt
  pthread
)

ament_target_dependencies(d1_joint_state_bridge rclcpp sensor_msgs)

install(
  DIRECTORY calibration config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES
    README.md
    QUICKSTART.md
    USAGE_CN.md
    KNOWN_ISSUES.md
    LICENSE-ASSETS.md
    SHA256SUMS
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS d1_joint_state_bridge
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_hardware_profile test/test_hardware_profile.py)
endif()

ament_package()
