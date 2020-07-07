cmake_minimum_required(VERSION 2.8.3)
project(decomp_ros_msgs)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w -g -Wno-deprecated -fPIC")

find_package(catkin REQUIRED COMPONENTS
  #armadillo
  roscpp
  geometry_msgs
  message_generation
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    )

add_message_files(
  FILES
  Ellipsoid.msg
  EllipsoidArray.msg
  Polyhedron.msg
  PolyhedronArray.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
)

catkin_package(
  LIBRARIES encode_msgs decode_msgs
  CATKIN_DEPENDS message_runtime
)

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
