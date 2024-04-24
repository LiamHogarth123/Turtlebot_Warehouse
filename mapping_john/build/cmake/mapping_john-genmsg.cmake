# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mapping_john: 1 messages, 0 services")

set(MSG_I_FLAGS "-Imapping_john:/home/john/Desktop/Turtlebot_Warehouse/mapping_john/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mapping_john_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/john/Desktop/Turtlebot_Warehouse/mapping_john/msg/marker.msg" NAME_WE)
add_custom_target(_mapping_john_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mapping_john" "/home/john/Desktop/Turtlebot_Warehouse/mapping_john/msg/marker.msg" "std_msgs/MultiArrayDimension:std_msgs/MultiArrayLayout:std_msgs/Float32MultiArray:std_msgs/Int16MultiArray"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mapping_john
  "/home/john/Desktop/Turtlebot_Warehouse/mapping_john/msg/marker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapping_john
)

### Generating Services

### Generating Module File
_generate_module_cpp(mapping_john
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapping_john
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mapping_john_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mapping_john_generate_messages mapping_john_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Desktop/Turtlebot_Warehouse/mapping_john/msg/marker.msg" NAME_WE)
add_dependencies(mapping_john_generate_messages_cpp _mapping_john_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapping_john_gencpp)
add_dependencies(mapping_john_gencpp mapping_john_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapping_john_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mapping_john
  "/home/john/Desktop/Turtlebot_Warehouse/mapping_john/msg/marker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapping_john
)

### Generating Services

### Generating Module File
_generate_module_eus(mapping_john
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapping_john
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mapping_john_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mapping_john_generate_messages mapping_john_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Desktop/Turtlebot_Warehouse/mapping_john/msg/marker.msg" NAME_WE)
add_dependencies(mapping_john_generate_messages_eus _mapping_john_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapping_john_geneus)
add_dependencies(mapping_john_geneus mapping_john_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapping_john_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mapping_john
  "/home/john/Desktop/Turtlebot_Warehouse/mapping_john/msg/marker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapping_john
)

### Generating Services

### Generating Module File
_generate_module_lisp(mapping_john
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapping_john
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mapping_john_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mapping_john_generate_messages mapping_john_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Desktop/Turtlebot_Warehouse/mapping_john/msg/marker.msg" NAME_WE)
add_dependencies(mapping_john_generate_messages_lisp _mapping_john_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapping_john_genlisp)
add_dependencies(mapping_john_genlisp mapping_john_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapping_john_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mapping_john
  "/home/john/Desktop/Turtlebot_Warehouse/mapping_john/msg/marker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapping_john
)

### Generating Services

### Generating Module File
_generate_module_nodejs(mapping_john
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapping_john
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mapping_john_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mapping_john_generate_messages mapping_john_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Desktop/Turtlebot_Warehouse/mapping_john/msg/marker.msg" NAME_WE)
add_dependencies(mapping_john_generate_messages_nodejs _mapping_john_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapping_john_gennodejs)
add_dependencies(mapping_john_gennodejs mapping_john_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapping_john_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mapping_john
  "/home/john/Desktop/Turtlebot_Warehouse/mapping_john/msg/marker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapping_john
)

### Generating Services

### Generating Module File
_generate_module_py(mapping_john
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapping_john
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mapping_john_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mapping_john_generate_messages mapping_john_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Desktop/Turtlebot_Warehouse/mapping_john/msg/marker.msg" NAME_WE)
add_dependencies(mapping_john_generate_messages_py _mapping_john_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapping_john_genpy)
add_dependencies(mapping_john_genpy mapping_john_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapping_john_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapping_john)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapping_john
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mapping_john_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapping_john)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapping_john
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mapping_john_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapping_john)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapping_john
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mapping_john_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapping_john)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapping_john
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mapping_john_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapping_john)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapping_john\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapping_john
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mapping_john_generate_messages_py std_msgs_generate_messages_py)
endif()
