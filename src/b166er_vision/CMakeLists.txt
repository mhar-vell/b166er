cmake_minimum_required(VERSION 3.5)
project(b166er_vision)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)

catkin_install_python(
  PROGRAMS
    nodes/apriltag_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
