cmake_minimum_required(VERSION 3.0.2)
project(sim_manager)

set(CMAKE_BUILD_TYPE Release)

set(PYTHON_EXECUTABLE /usr/bin/python3)
set(PYTHON_INCLUDE_DIR /usr/include/python3.6m)
set(PYTHON_LIBRARY /usr/lib/x86_64-linux-gnu/libpython3.6m.so)

add_definitions(-DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE})
add_definitions(-DPYTHON_INCLUDE_DIR=${PYTHON_INCLUDE_DIR})
add_definitions(-DPYTHON_LIBRARY=${PYTHON_LIBRARY})


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
