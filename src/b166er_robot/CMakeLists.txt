cmake_minimum_required(VERSION 3.5)
project(b166er_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  urdf
  xacro
  gazebo_ros
  gazebo_ros_control
  robot_state_publisher
  joint_state_publisher
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs urdf xacro robot_state_publisher joint_state_publisher
)

include_directories(${catkin_INCLUDE_DIRS})

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro"
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  FILES_MATCHING PATTERN "*.world"
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.rviz"
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/imu_monitor.py
  scripts/stability_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)