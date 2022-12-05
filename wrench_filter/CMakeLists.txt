cmake_minimum_required(VERSION 3.0.2)
project(wrench_filter)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  cpp_filters
  roscpp
  dynamic_reconfigure)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3)

generate_dynamic_reconfigure_options(
  cfg/WrenchFilter.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wrench_filter
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS EIGEN
)

###########
## Build ##
###########

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/wrench_filter.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN_LIBRARIES}
  )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

