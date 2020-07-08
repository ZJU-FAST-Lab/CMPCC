cmake_minimum_required(VERSION 2.8.3)
project(cmpcc)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w -g -fPIC")

find_package(catkin REQUIRED COMPONENTS
        message_generation
        message_runtime
        # decomp_ros_msgs
        quadrotor_msgs
        roscpp
        roslib
        )

find_package(Eigen3)
find_package(yaml-cpp)
find_package(osqp REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(osqp_test
    src/osqp_test.cpp
    src/osqp_interface.cpp
)
target_link_libraries(osqp_test osqp::osqp)

add_executable(corridor_test
    src/corridor_test.cpp
    src/map.cpp
    src/bezier_base.cpp
    src/my_spline.cpp
    src/corridor.cpp)
add_dependencies(corridor_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(corridor_test ${catkin_LIBRARIES} yaml-cpp)

add_executable(solver_test
    src/display_msg.cpp
    src/solver_test.cpp
    src/mpc_solver.cpp
    src/osqp_interface.cpp
    src/model.cpp
    src/map.cpp
    src/bezier_base.cpp
    src/my_spline.cpp
    src/corridor.cpp)
add_dependencies(solver_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(solver_test
    ${catkin_LIBRARIES} 
    yaml-cpp
    osqp::osqp
)

add_executable(simulation_node
    src/simulation_node.cpp
    src/display_msg.cpp
    src/mpc_solver.cpp
    src/osqp_interface.cpp
    src/model.cpp
    src/map.cpp
    src/bezier_base.cpp
    src/my_spline.cpp
    src/corridor.cpp) 
add_dependencies(simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulation_node
    ${catkin_LIBRARIES}
    yaml-cpp
    osqp::osqp
)   

add_executable(flyfly_node
    src/flyfly_node.cpp
    src/display_msg.cpp
    src/mpc_solver.cpp
    src/osqp_interface.cpp
    src/model.cpp
    src/map.cpp
    src/bezier_base.cpp
    src/my_spline.cpp
    src/corridor.cpp) 
add_dependencies(flyfly_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flyfly_node
    ${catkin_LIBRARIES}
    yaml-cpp
    osqp::osqp
)   

add_executable(plot_bag_node
    src/plot_bag_node.cpp
    src/display_msg.cpp
    src/mpc_solver.cpp
    src/osqp_interface.cpp
    src/model.cpp
    src/map.cpp
    src/bezier_base.cpp
    src/my_spline.cpp
    src/corridor.cpp) 
add_dependencies(plot_bag_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(plot_bag_node
    ${catkin_LIBRARIES}
    yaml-cpp
    osqp::osqp
)   

add_executable(fly_trr_node
    src/fly_trr_node.cpp
    src/display_msg.cpp
    src/mpc_solver.cpp
    src/osqp_interface.cpp
    src/model.cpp
    src/map.cpp
    src/bezier_base.cpp
    src/my_spline.cpp
    src/corridor.cpp) 
add_dependencies(fly_trr_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fly_trr_node
    ${catkin_LIBRARIES}
    yaml-cpp
    osqp::osqp
)   

