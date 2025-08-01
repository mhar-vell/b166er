# Install script for directory: /home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/install")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/movemaster_msg/msg" TYPE FILE FILES
    "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg/setpoint.msg"
    "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg/status.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/movemaster_msg/cmake" TYPE FILE FILES "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/build/movemaster_msg/catkin_generated/installspace/movemaster_msg-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/devel/include/movemaster_msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/root/.matlab/R2021a/ros1/glnxa64/venv/bin/python2" -m compileall "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/devel/lib/python2.7/dist-packages/movemaster_msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/devel/lib/python2.7/dist-packages/movemaster_msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/build/movemaster_msg/catkin_generated/installspace/movemaster_msg.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/movemaster_msg/cmake" TYPE FILE FILES "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/build/movemaster_msg/catkin_generated/installspace/movemaster_msg-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/movemaster_msg/cmake" TYPE FILE FILES
    "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/build/movemaster_msg/catkin_generated/installspace/movemaster_msgConfig.cmake"
    "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/build/movemaster_msg/catkin_generated/installspace/movemaster_msgConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/movemaster_msg" TYPE FILE FILES "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/devel/lib/libmovemaster_msg_matlab.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmovemaster_msg_matlab.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmovemaster_msg_matlab.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmovemaster_msg_matlab.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/m/" TYPE DIRECTORY FILES "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/m/" FILES_MATCHING REGEX "/[^/]*\\.m$")
endif()

