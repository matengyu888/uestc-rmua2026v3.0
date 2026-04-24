cmake_minimum_required(VERSION 3.0.2)
project(quadrotor_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  PositionCommand.msg
  MpcPositionCommand.msg
  PolynomialTrajectory.msg
  QuadrotorState.msg
  TrakingPerformance.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs std_msgs
)
