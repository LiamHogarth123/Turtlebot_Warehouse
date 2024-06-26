cmake_minimum_required(VERSION 3.0.1)
project(final_verison_attempt2)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  rosbag
  roslib
  message_generation
  marker_msgs
  tf
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Set compiler flags for deprecation warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
message(STATUS "CMake C++ FLAGS: ${CMAKE_CXX_FLAGS}")

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs nav_msgs rosbag roslib message_runtime marker_msgs tf
  DEPENDS EIGEN3 OpenCV
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Add libraries
add_library(${PROJECT_NAME}_prm src/prm.cpp)
add_library(${PROJECT_NAME}_taskAlloction src/taskAlloction.cpp)
add_library(${PROJECT_NAME}_turtlebot src/turtlebot.cpp)
add_library(${PROJECT_NAME}_control src/control.cpp src/sensorprocessing.cpp)
add_library(${PROJECT_NAME}_path_avoidance src/path_avoidance.cpp)

# Link libraries with catkin libraries
target_link_libraries(${PROJECT_NAME}_prm
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_taskAlloction
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_turtlebot
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_control
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_path_avoidance
  ${catkin_LIBRARIES}
)

# Add executable target
add_executable(${PROJECT_NAME}_method src/main.cpp src/method.cpp)

# Link executable with the necessary libraries
target_link_libraries(${PROJECT_NAME}_method
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PROJECT_NAME}_prm
  ${PROJECT_NAME}_taskAlloction
  ${PROJECT_NAME}_control
  ${PROJECT_NAME}_turtlebot
  ${PROJECT_NAME}_path_avoidance
)

# Optional: Link against pthreads if threading issues arise
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME}_method Threads::Threads)
