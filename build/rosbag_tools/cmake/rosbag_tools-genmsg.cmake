# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rosbag_tools: 2 messages, 0 services")

set(MSG_I_FLAGS "-Irosbag_tools:/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rosbag_tools_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomMsg.msg" NAME_WE)
add_custom_target(_rosbag_tools_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rosbag_tools" "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomMsg.msg" "std_msgs/Header:rosbag_tools/CustomPoint"
)

get_filename_component(_filename "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg" NAME_WE)
add_custom_target(_rosbag_tools_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rosbag_tools" "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rosbag_tools
  "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosbag_tools
)
_generate_msg_cpp(rosbag_tools
  "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosbag_tools
)

### Generating Services

### Generating Module File
_generate_module_cpp(rosbag_tools
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosbag_tools
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rosbag_tools_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rosbag_tools_generate_messages rosbag_tools_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomMsg.msg" NAME_WE)
add_dependencies(rosbag_tools_generate_messages_cpp _rosbag_tools_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg" NAME_WE)
add_dependencies(rosbag_tools_generate_messages_cpp _rosbag_tools_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosbag_tools_gencpp)
add_dependencies(rosbag_tools_gencpp rosbag_tools_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosbag_tools_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rosbag_tools
  "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosbag_tools
)
_generate_msg_eus(rosbag_tools
  "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosbag_tools
)

### Generating Services

### Generating Module File
_generate_module_eus(rosbag_tools
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosbag_tools
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rosbag_tools_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rosbag_tools_generate_messages rosbag_tools_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomMsg.msg" NAME_WE)
add_dependencies(rosbag_tools_generate_messages_eus _rosbag_tools_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg" NAME_WE)
add_dependencies(rosbag_tools_generate_messages_eus _rosbag_tools_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosbag_tools_geneus)
add_dependencies(rosbag_tools_geneus rosbag_tools_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosbag_tools_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rosbag_tools
  "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosbag_tools
)
_generate_msg_lisp(rosbag_tools
  "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosbag_tools
)

### Generating Services

### Generating Module File
_generate_module_lisp(rosbag_tools
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosbag_tools
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rosbag_tools_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rosbag_tools_generate_messages rosbag_tools_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomMsg.msg" NAME_WE)
add_dependencies(rosbag_tools_generate_messages_lisp _rosbag_tools_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg" NAME_WE)
add_dependencies(rosbag_tools_generate_messages_lisp _rosbag_tools_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosbag_tools_genlisp)
add_dependencies(rosbag_tools_genlisp rosbag_tools_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosbag_tools_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rosbag_tools
  "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosbag_tools
)
_generate_msg_nodejs(rosbag_tools
  "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosbag_tools
)

### Generating Services

### Generating Module File
_generate_module_nodejs(rosbag_tools
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosbag_tools
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rosbag_tools_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rosbag_tools_generate_messages rosbag_tools_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomMsg.msg" NAME_WE)
add_dependencies(rosbag_tools_generate_messages_nodejs _rosbag_tools_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg" NAME_WE)
add_dependencies(rosbag_tools_generate_messages_nodejs _rosbag_tools_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosbag_tools_gennodejs)
add_dependencies(rosbag_tools_gennodejs rosbag_tools_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosbag_tools_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rosbag_tools
  "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosbag_tools
)
_generate_msg_py(rosbag_tools
  "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosbag_tools
)

### Generating Services

### Generating Module File
_generate_module_py(rosbag_tools
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosbag_tools
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rosbag_tools_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rosbag_tools_generate_messages rosbag_tools_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomMsg.msg" NAME_WE)
add_dependencies(rosbag_tools_generate_messages_py _rosbag_tools_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dearmoon/projects/rosbag_tools/src/rosbag_tools/msg/CustomPoint.msg" NAME_WE)
add_dependencies(rosbag_tools_generate_messages_py _rosbag_tools_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosbag_tools_genpy)
add_dependencies(rosbag_tools_genpy rosbag_tools_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosbag_tools_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosbag_tools)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosbag_tools
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rosbag_tools_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(rosbag_tools_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosbag_tools)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosbag_tools
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rosbag_tools_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(rosbag_tools_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosbag_tools)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosbag_tools
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rosbag_tools_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(rosbag_tools_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosbag_tools)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosbag_tools
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rosbag_tools_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(rosbag_tools_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosbag_tools)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosbag_tools\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosbag_tools
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rosbag_tools_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(rosbag_tools_generate_messages_py sensor_msgs_generate_messages_py)
endif()
