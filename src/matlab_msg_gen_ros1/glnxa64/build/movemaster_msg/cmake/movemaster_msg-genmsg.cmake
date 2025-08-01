# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "movemaster_msg: 2 messages, 0 services")

set(MSG_I_FLAGS "-Imovemaster_msg:/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg;-Istd_msgs:/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(movemaster_msg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg/setpoint.msg" NAME_WE)
add_custom_target(_movemaster_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "movemaster_msg" "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg/setpoint.msg" ""
)

get_filename_component(_filename "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg/status.msg" NAME_WE)
add_custom_target(_movemaster_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "movemaster_msg" "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg/status.msg" ""
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(movemaster_msg
  "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg/setpoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/movemaster_msg
)
_generate_msg_cpp(movemaster_msg
  "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg/status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/movemaster_msg
)

### Generating Services

### Generating Module File
_generate_module_cpp(movemaster_msg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/movemaster_msg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(movemaster_msg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(movemaster_msg_generate_messages movemaster_msg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg/setpoint.msg" NAME_WE)
add_dependencies(movemaster_msg_generate_messages_cpp _movemaster_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg/status.msg" NAME_WE)
add_dependencies(movemaster_msg_generate_messages_cpp _movemaster_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(movemaster_msg_gencpp)
add_dependencies(movemaster_msg_gencpp movemaster_msg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS movemaster_msg_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(movemaster_msg
  "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg/setpoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movemaster_msg
)
_generate_msg_py(movemaster_msg
  "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg/status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movemaster_msg
)

### Generating Services

### Generating Module File
_generate_module_py(movemaster_msg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movemaster_msg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(movemaster_msg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(movemaster_msg_generate_messages movemaster_msg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg/setpoint.msg" NAME_WE)
add_dependencies(movemaster_msg_generate_messages_py _movemaster_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/movemaster/Movemaster_UTFPR/src/matlab_msg_gen_ros1/glnxa64/src/movemaster_msg/msg/status.msg" NAME_WE)
add_dependencies(movemaster_msg_generate_messages_py _movemaster_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(movemaster_msg_genpy)
add_dependencies(movemaster_msg_genpy movemaster_msg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS movemaster_msg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/movemaster_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/movemaster_msg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(movemaster_msg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movemaster_msg)
  install(CODE "execute_process(COMMAND \"/root/.matlab/R2021a/ros1/glnxa64/venv/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movemaster_msg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movemaster_msg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(movemaster_msg_generate_messages_py std_msgs_generate_messages_py)
endif()
