cmake_minimum_required(VERSION 2.8.3)
project(hrp2g_wheelbase)
set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED COMPONENTS
  roscpp message_generation)

add_service_files(
  FILES
  Gopose.srv)

generate_messages(
  DEPENDENCIES)

catkin_package(
  CATKIN_DEPENDS
)


#uav_teleop_keyboard node
add_executable(uav_teleop_keyboard src/teleop/uav_teleop_keyboard.cpp)
target_link_libraries(uav_teleop_keyboard ${catkin_LIBRARIES})
#uav_teleop_joy node
add_executable(uav_teleop_joy src/teleop/uav_teleop_joy.cpp)
target_link_libraries(uav_teleop_joy ${catkin_LIBRARIES})


#goblin_gopose node
add_executable(goblin_gopose src/gopose/goblin_gopose.cpp)
target_link_libraries(goblin_gopose ${catkin_LIBRARIES})




