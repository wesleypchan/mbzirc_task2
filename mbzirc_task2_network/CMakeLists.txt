cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_task2_network)

find_package(catkin REQUIRED
  jsk_network_tools
  std_msgs
  sensor_msgs
  message_generation
  image_transport
  image_geometry
  compressed_image_transport
  tf2_msgs
  jsk_recognition_msgs
  nav_msgs
)


add_message_files(
   FILES
   Remote_LargeDataHRP2.msg
   Remote_SmallDataHRP2.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   dynamic_reconfigure
   tf2_msgs
   jsk_recognition_msgs
   nav_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs dynamic_reconfigure
)