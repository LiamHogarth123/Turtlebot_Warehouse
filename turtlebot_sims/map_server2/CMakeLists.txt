cmake_minimum_required(VERSION 3.0.2)
project(map_server2)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            nav_msgs
            tf2
        )

find_package(Bullet REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp QUIET)
if(NOT YAMLCPP_FOUND)
  find_package(yaml-cpp 0.6 REQUIRED)
  set(YAMLCPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
  set(YAMLCPP_LIBRARIES ${YAML_CPP_LIBRARIES})
  add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
else()
  if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
  endif()
  link_directories(${YAMLCPP_LIBRARY_DIRS})
endif()

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        map_server2_image_loader
    CATKIN_DEPENDS
        roscpp
        nav_msgs
        tf2
)

include_directories(
    include
    ${BULLET_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${SDL_INCLUDE_DIR}
    ${SDL_IMAGE_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(map_server2_image_loader src/image_loader.cpp)
add_dependencies(map_server2_image_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_server2_image_loader
    ${BULLET_LIBRARIES}
    ${catkin_LIBRARIES}
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
)

add_executable(map_server2 src/main.cpp)
add_dependencies(map_server2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_server2
    map_server2_image_loader
    ${YAMLCPP_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(map_saver2 src/map_saver.cpp)
add_dependencies(map_saver2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(map_saver2 PROPERTIES OUTPUT_NAME map_saver2)
target_link_libraries(map_saver2
    ${catkin_LIBRARIES}
)

# copy test data to same place as tests are run
function(copy_test_data)
    cmake_parse_arguments(PROJECT "" "" "FILES" ${ARGN})
    foreach(datafile ${PROJECT_FILES})
        file(COPY ${datafile} DESTINATION ${PROJECT_BINARY_DIR}/test)
    endforeach()
endfunction()

## Tests
if(CATKIN_ENABLE_TESTING)
  copy_test_data( FILES
      test/testmap.bmp
      test/testmap.png
      test/spectrum.png )
  catkin_add_gtest(${PROJECT_NAME}_utest test/utest.cpp test/test_constants.cpp)
  target_link_libraries(${PROJECT_NAME}_utest
    map_server2_image_loader
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
  )

  find_package(roslib REQUIRED)
  include_directories(${roslib_INCLUDE_DIRS})
  add_executable(rtest2 test/rtest.cpp test/test_constants.cpp)
  add_dependencies(rtest2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(tests rtest2)
  target_link_libraries( rtest2
      ${GTEST_LIBRARIES}
      ${catkin_LIBRARIES}
      ${roslib_LIBRARIES}
  )

  # This has to be done after we've already built targets, or catkin variables get borked
  find_package(rostest REQUIRED)
  add_rostest(test/rtest.xml)
endif()

## Install executables and/or libraries
install(TARGETS map_saver2 map_server2
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS map_server2_image_loader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

## Install executable python script
install(
    PROGRAMS
      scripts/crop_map
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
