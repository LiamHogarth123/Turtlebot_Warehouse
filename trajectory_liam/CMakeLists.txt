cmake_minimum_required(VERSION 3.0.1)
project(trajectory_liam)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  rosbag
  roslib
  message_generation  # Add this line for message generation
  # OpenCV REQUIRED  
  

)
# OpenCV
# include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
message(STATUS "CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

catkin_package()

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  # ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_method src/main.cpp src/method.cpp)

add_dependencies(${PROJECT_NAME}_method ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(mylib src/readMap.cpp)

target_link_libraries(${PROJECT_NAME}_method ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_method mylib)

# target_link_libraries(${PROJECT_NAME}_method ${OpenCV_LIBS})
