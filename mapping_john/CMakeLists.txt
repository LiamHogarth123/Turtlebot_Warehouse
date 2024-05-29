cmake_minimum_required(VERSION 3.0.1)
project(coord_converter)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  rosbag
  roslib
)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
message(STATUS "CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")


catkin_package()

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_method src/main.cpp)

add_dependencies(${PROJECT_NAME}_method ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_method ${catkin_LIBRARIES}

# Install the launch file
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)