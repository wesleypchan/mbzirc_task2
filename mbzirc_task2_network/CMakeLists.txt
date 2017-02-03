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
  trajectory_msgs
)


add_message_files(
   FILES
   HRP22Remote_LargeData.msg
   HRP22Remote_SmallData.msg
   AERO2Remote_LargeData.msg
   Remote2AERO.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   dynamic_reconfigure
   tf2_msgs
   jsk_recognition_msgs
   nav_msgs
   trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs dynamic_reconfigure
)