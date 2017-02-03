cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_task2_utils)

find_package(catkin REQUIRED COMPONENTS
  jsk_tools
  mbzirc_task2_network
  roscpp
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mbzirc_task2_utils
  CATKIN_DEPENDS jsk_tools mbzirc_task2_network roscpp
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_add_env_hooks(99.mbzirc_task2_utils SHELLS bash zsh DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)