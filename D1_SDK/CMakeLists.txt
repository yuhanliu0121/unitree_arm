project(marm_code)
cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)
# ROS 2 Humble may prepend an ABI-incompatible Cyclone DDS C library through
# LD_LIBRARY_PATH. DT_RPATH must take precedence so every SDK executable uses
# the matching /usr/local C and C++ libraries.
set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags,-rpath,/usr/local/lib")

add_executable(joint_angle_control src/joint_angle_control.cpp src/msg/ArmString_.cpp)
add_executable(multiple_joint_angle_control src/multiple_joint_angle_control.cpp src/msg/ArmString_.cpp)
add_executable(joint_enable_control src/joint_enable_control.cpp src/msg/ArmString_.cpp)
add_executable(arm_zero_control src/arm_zero_control.cpp src/msg/ArmString_.cpp)
add_executable(
    arm_stowed_control
    src/arm_stowed_control.cpp
    src/msg/ArmString_.cpp
    src/msg/PubServoInfo_.cpp)
add_executable(all_joints_unload src/all_joints_unload.cpp src/msg/ArmString_.cpp)
add_executable(get_arm_joint_angle src/get_arm_joint_angle.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp)
