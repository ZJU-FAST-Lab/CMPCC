cmake_minimum_required(VERSION 2.8.3)
project(plan_env)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  message_filters
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

message(STATUS "${OpenCV_INCLUDE_DIRS}")
message(STATUS "${PCL_INCLUDE_DIRS}")

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs 
#  DEPENDS system_lib
)

include_directories( 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARIES})


# add_library( plan_env 
#     src/sdf_map.cpp 
#     src/edt_environment.cpp 
#     src/raycast.cpp
# )
# target_link_libraries( plan_env
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
#     ${OpenCV_LIBRARIES}
# ) 

add_executable( plan_env
    src/plan_env_node.cpp
    src/sdf_map.cpp 
    src/raycast.cpp
)
target_link_libraries( plan_env
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)  
