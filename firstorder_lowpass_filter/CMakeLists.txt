cmake_minimum_required(VERSION 3.0.2)
project(firstorder_lowpass_filter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  geometry_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES firstorder_lowpass_filter
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(jointstates_filter src/jointstates_filter.cpp)
target_link_libraries(jointstates_filter
  ${catkin_LIBRARIES}
  )

add_executable(wrenchstamped_filter src/wrenchstamped_filter.cpp)
target_link_libraries(wrenchstamped_filter
  ${catkin_LIBRARIES}
  )

install(TARGETS jointstates_filter wrenchstamped_filter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_firstorder_lowpass_filter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
