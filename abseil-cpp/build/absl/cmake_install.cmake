# Install script for directory: /home/clark/catkin_ws/abseil-cpp/absl

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local/stow/absl")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/clark/catkin_ws/abseil-cpp/build/absl/base/cmake_install.cmake")
  include("/home/clark/catkin_ws/abseil-cpp/build/absl/algorithm/cmake_install.cmake")
  include("/home/clark/catkin_ws/abseil-cpp/build/absl/container/cmake_install.cmake")
  include("/home/clark/catkin_ws/abseil-cpp/build/absl/debugging/cmake_install.cmake")
  include("/home/clark/catkin_ws/abseil-cpp/build/absl/hash/cmake_install.cmake")
  include("/home/clark/catkin_ws/abseil-cpp/build/absl/memory/cmake_install.cmake")
  include("/home/clark/catkin_ws/abseil-cpp/build/absl/meta/cmake_install.cmake")
  include("/home/clark/catkin_ws/abseil-cpp/build/absl/numeric/cmake_install.cmake")
  include("/home/clark/catkin_ws/abseil-cpp/build/absl/strings/cmake_install.cmake")
  include("/home/clark/catkin_ws/abseil-cpp/build/absl/synchronization/cmake_install.cmake")
  include("/home/clark/catkin_ws/abseil-cpp/build/absl/time/cmake_install.cmake")
  include("/home/clark/catkin_ws/abseil-cpp/build/absl/types/cmake_install.cmake")
  include("/home/clark/catkin_ws/abseil-cpp/build/absl/utility/cmake_install.cmake")

endif()

