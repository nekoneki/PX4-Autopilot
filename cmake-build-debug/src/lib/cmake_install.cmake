# Install script for directory: /home/tang/PX4-Autopilot/src/lib

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/airspeed/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/avoidance/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/battery/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/bezier/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/cdev/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/circuit_breaker/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/collision_prevention/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/component_information/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/controllib/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/conversion/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/crypto/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/drivers/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/field_sensor_bias_estimator/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/geo/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/hysteresis/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/l1/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/landing_slope/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/led/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/matrix/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/mathlib/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/mixer/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/mixer_module/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/motion_planning/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/npfg/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/perf/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/pid/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/pid_design/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/pwm/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/rc/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/sensor_calibration/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/slew_rate/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/systemlib/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/system_identification/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/tecs/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/terrain_estimation/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/tunes/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/version/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/weather_vane/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/wind_estimator/cmake_install.cmake")
  include("/home/tang/PX4-Autopilot/cmake-build-debug/src/lib/world_magnetic_model/cmake_install.cmake")

endif()

