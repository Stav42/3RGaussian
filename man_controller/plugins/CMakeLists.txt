cmake_minimum_required(VERSION 3.10)

project(shvan_plugins VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)


find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
#find_package(roscpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  shvan_msgs
)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(include ${GAZEBO_INCLUDE_DIRS})# /usr/local/MATLAB/R2021a/extern/include)
include_directories(${catkin_INCLUDE_DIRS})

set(INCLUDE_DIR "include")

link_directories(${GAZEBO_LIBRARY_DIRS}) # /usr/local/MATLAB/R2021a/bin/glnxa64)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(motor_plugin SHARED src/motor_control.cc)
target_link_libraries(motor_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} Eigen3::Eigen)

