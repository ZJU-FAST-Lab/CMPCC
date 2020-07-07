cmake_minimum_required(VERSION 2.8.3)
project(catkin_simple)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w -Wno-deprecated -g -fPIC")

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS catkin
  CFG_EXTRAS catkin_simple-extras.cmake
)
