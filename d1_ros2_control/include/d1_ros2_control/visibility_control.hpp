#ifndef D1_ROS2_CONTROL__VISIBILITY_CONTROL_HPP_
#define D1_ROS2_CONTROL__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define D1_ROS2_CONTROL_EXPORT __attribute__((dllexport))
    #define D1_ROS2_CONTROL_IMPORT __attribute__((dllimport))
  #else
    #define D1_ROS2_CONTROL_EXPORT __declspec(dllexport)
    #define D1_ROS2_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef D1_ROS2_CONTROL_BUILDING_DLL
    #define D1_ROS2_CONTROL_PUBLIC D1_ROS2_CONTROL_EXPORT
  #else
    #define D1_ROS2_CONTROL_PUBLIC D1_ROS2_CONTROL_IMPORT
  #endif
#else
  #define D1_ROS2_CONTROL_EXPORT __attribute__((visibility("default")))
  #define D1_ROS2_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define D1_ROS2_CONTROL_PUBLIC __attribute__((visibility("default")))
  #else
    #define D1_ROS2_CONTROL_PUBLIC
  #endif
#endif

#endif  // D1_ROS2_CONTROL__VISIBILITY_CONTROL_HPP_
