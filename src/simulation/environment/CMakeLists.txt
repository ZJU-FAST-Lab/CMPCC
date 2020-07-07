cmake_minimum_required(VERSION 2.8.3)
project(environment)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -w") 

add_executable ( generate_map_node 
    src/generate_map_node.cpp )

target_link_libraries( generate_map_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} ) 

