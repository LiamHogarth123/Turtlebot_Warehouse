################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(multi_nav)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  visualization_msgs
  geometry_msgs
  std_msgs
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package()

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)

add_executable(global_position2 src/global_position2.cpp)

target_link_libraries(global_position2
  ${catkin_LIBRARIES}
)

add_executable(global_position3 src/global_position3.cpp)

target_link_libraries(global_position3
  ${catkin_LIBRARIES}
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################
install(DIRECTORY launch maps param rviz src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
