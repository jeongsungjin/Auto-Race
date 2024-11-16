# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "aruco_detection: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iaruco_detection:/home/wego/Auto-Race/src/aruco_detection/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(aruco_detection_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wego/Auto-Race/src/aruco_detection/msg/MarkerArray.msg" NAME_WE)
add_custom_target(_aruco_detection_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "aruco_detection" "/home/wego/Auto-Race/src/aruco_detection/msg/MarkerArray.msg" "geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Pose:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(aruco_detection
  "/home/wego/Auto-Race/src/aruco_detection/msg/MarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/aruco_detection
)

### Generating Services

### Generating Module File
_generate_module_cpp(aruco_detection
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/aruco_detection
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(aruco_detection_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(aruco_detection_generate_messages aruco_detection_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wego/Auto-Race/src/aruco_detection/msg/MarkerArray.msg" NAME_WE)
add_dependencies(aruco_detection_generate_messages_cpp _aruco_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(aruco_detection_gencpp)
add_dependencies(aruco_detection_gencpp aruco_detection_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aruco_detection_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(aruco_detection
  "/home/wego/Auto-Race/src/aruco_detection/msg/MarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/aruco_detection
)

### Generating Services

### Generating Module File
_generate_module_eus(aruco_detection
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/aruco_detection
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(aruco_detection_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(aruco_detection_generate_messages aruco_detection_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wego/Auto-Race/src/aruco_detection/msg/MarkerArray.msg" NAME_WE)
add_dependencies(aruco_detection_generate_messages_eus _aruco_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(aruco_detection_geneus)
add_dependencies(aruco_detection_geneus aruco_detection_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aruco_detection_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(aruco_detection
  "/home/wego/Auto-Race/src/aruco_detection/msg/MarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/aruco_detection
)

### Generating Services

### Generating Module File
_generate_module_lisp(aruco_detection
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/aruco_detection
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(aruco_detection_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(aruco_detection_generate_messages aruco_detection_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wego/Auto-Race/src/aruco_detection/msg/MarkerArray.msg" NAME_WE)
add_dependencies(aruco_detection_generate_messages_lisp _aruco_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(aruco_detection_genlisp)
add_dependencies(aruco_detection_genlisp aruco_detection_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aruco_detection_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(aruco_detection
  "/home/wego/Auto-Race/src/aruco_detection/msg/MarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/aruco_detection
)

### Generating Services

### Generating Module File
_generate_module_nodejs(aruco_detection
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/aruco_detection
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(aruco_detection_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(aruco_detection_generate_messages aruco_detection_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wego/Auto-Race/src/aruco_detection/msg/MarkerArray.msg" NAME_WE)
add_dependencies(aruco_detection_generate_messages_nodejs _aruco_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(aruco_detection_gennodejs)
add_dependencies(aruco_detection_gennodejs aruco_detection_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aruco_detection_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(aruco_detection
  "/home/wego/Auto-Race/src/aruco_detection/msg/MarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aruco_detection
)

### Generating Services

### Generating Module File
_generate_module_py(aruco_detection
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aruco_detection
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(aruco_detection_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(aruco_detection_generate_messages aruco_detection_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wego/Auto-Race/src/aruco_detection/msg/MarkerArray.msg" NAME_WE)
add_dependencies(aruco_detection_generate_messages_py _aruco_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(aruco_detection_genpy)
add_dependencies(aruco_detection_genpy aruco_detection_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aruco_detection_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/aruco_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/aruco_detection
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(aruco_detection_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(aruco_detection_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/aruco_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/aruco_detection
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(aruco_detection_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(aruco_detection_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/aruco_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/aruco_detection
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(aruco_detection_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(aruco_detection_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/aruco_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/aruco_detection
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(aruco_detection_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(aruco_detection_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aruco_detection)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aruco_detection\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aruco_detection
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(aruco_detection_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(aruco_detection_generate_messages_py geometry_msgs_generate_messages_py)
endif()
