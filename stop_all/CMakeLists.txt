cmake_minimum_required(VERSION 3.0.1)
project(stop_all)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  rosbag
  roslib
  message_generation
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(yaml-cpp REQUIRED) # Ensure yaml-cpp is found

# Set compiler flags for deprecation warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
message(STATUS "CMake C++ FLAGS: ${CMAKE_CXX_FLAGS}")

# Declare a catkin package
catkin_package()

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  # ${YAML_CPP_INCLUDE_DIR} # Include the yaml-cpp headers
)

# Add executable target
add_executable(${PROJECT_NAME}_method src/main.cpp src/method.cpp)

# Link libraries with the executable
target_link_libraries(${PROJECT_NAME}_method
  ${catkin_LIBRARIES}
  # ${YAML_CPP_INCLUDE_DIR}# Link against yaml-cpp (no need for manual path specification)

)


