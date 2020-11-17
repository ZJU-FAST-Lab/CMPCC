cmake_minimum_required(VERSION 3.0.2)
project(quadrotor_msgs)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  PositionCommand.msg
  SO3Command.msg
  AuxCommand.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)