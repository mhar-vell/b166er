cmake_minimum_required(VERSION 3.5)
project(b166er_whole_body_control)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

add_message_files(
  FILES
  RobotState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2_ros
    tf2_geometry_msgs
)

catkin_install_python(PROGRAMS
  nodes/state_estimator.py
  nodes/whole_body_planner.py
  nodes/fuzzy_wb_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
