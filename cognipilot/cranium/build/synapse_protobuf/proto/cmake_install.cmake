# Install script for directory: /home/user/cognipilot/cranium/src/synapse_protobuf/proto

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/user/cognipilot/cranium/install/synapse_protobuf")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/libsynapse_protobuf.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/synapse_protobuf" TYPE FILE FILES
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/actuators.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/altimeter.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/battery_state.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/bezier_curve.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/bezier_trajectory.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/can.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/header.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/imu.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/joy.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/led.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/led_array.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/magnetic_field.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/nav_sat_fix.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/nav_sat_status.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/odometry.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/point.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/pose.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/pose_with_covariance.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/quaternion.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/safety.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/sim_clock.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/status.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/time.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/twist.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/twist_with_covariance.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/vector3.pb.h"
    "/home/user/cognipilot/cranium/build/synapse_protobuf/proto/wheel_odometry.pb.h"
    )
endif()

