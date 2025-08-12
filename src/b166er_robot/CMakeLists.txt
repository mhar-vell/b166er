cmake_minimum_required(VERSION 3.5)
project(b166er_robot)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  robot_state_publisher
  joint_state_publisher
  message_generation
  actionlib_msgs
)

## Generate messages
add_message_files(
  FILES
  RobotStatus.msg
  StabilityMetrics.msg
)

## Generate services
add_service_files(
  FILES
  CalibrateIMU.srv
  SetStabilityMode.srv
)

## Generate actions
add_action_files(
  FILES
  MoveToPosition.action
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
)

## Catkin package configuration
catkin_package(
  CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    message_runtime
)

## Build C++ nodes (if any)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/imu_monitor.py
  scripts/stability_controller.py
  scripts/arm_controller.py
  scripts/navigation_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

## Install URDF files
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

## Install mesh files
install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

## Install RViz configs
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

## Install world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)