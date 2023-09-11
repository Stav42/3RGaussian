cmake_minimum_required(VERSION 3.10)

project(shvan_plugins VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)


find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(drake CONFIG REQUIRED HINTS "/Users/stav.42/drake/lib/cmake/drake")
find_package(fmt REQUIRED)
#find_package(roscpp REQUIRED)

set(drake_INCLUDE_DIRS "/Users/stav.42/drake/include/")
include_directories(${drake_INCLUDE_DIRS})
link_directories("/Users/stav.42/drake/lib/")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  man_controller
)

find_package(Boost REQUIRED COMPONENTS system)


include_directories(${roscpp_INCLUDE_DIRS})
include_directories(include ${GAZEBO_INCLUDE_DIRS}   ${Boost_INCLUDE_DIRS})# /usr/local/MATLAB/R2021a/extern/include)
include_directories(include ${catkin_INCLUDE_DIRS} )
include_directories(include ${CMAKE_CURRENT_SOURCE_DIR}/../include ${MAN_CONTROLLER_INCLUDE_DIRS})


set(INCLUDE_DIR "include")

link_directories(${GAZEBO_LIBRARY_DIRS}) # /usr/local/MATLAB/R2021a/bin/glnxa64)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(motor_plugin SHARED src/motor_control_dist.cc src/ff_torque.cpp src/gp.cpp)
# target_link_libraries(motor_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} Eigen3::Eigen)
target_link_libraries(motor_plugin
${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} Eigen3::Eigen ${catkin_LIBRARIES} fmt::fmt drake
)

# add_dependencies(motor_plugin ${})

