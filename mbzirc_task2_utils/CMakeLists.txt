cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_task2_utils)

find_package(catkin REQUIRED COMPONENTS
  jsk_tools
  mbzirc_task2_network
  roscpp
  nodelet
  nodelet_topic_tools
)
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES mbzirc_task2_utils
  CATKIN_DEPENDS jsk_tools mbzirc_task2_network roscpp
  #DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(image_throttle src/image_throttle.cpp)
target_link_libraries(image_throttle ${catkin_LIBRARIES})

catkin_add_env_hooks(99.mbzirc_task2_utils SHELLS bash zsh DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(PROGRAMS
  nodes/zed-odom-util.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

