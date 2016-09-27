cmake_minimum_required(VERSION 2.8.3)
project(ann4smc)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  rosbag
  baxter_core_msgs
  baxter_interface
  image_transport
)

catkin_python_setup()

#add_message_files(DIRECTORY msg FILES
#)

add_service_files(DIRECTORY srv FILES
  String.srv
  RL_environment/Start.srv
  RL_environment/Step.srv
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  rospy
  roscpp
  sensor_msgs
  std_msgs
  rosbag
  baxter_core_msgs
  baxter_interface
  image_transport
  cv_bridge
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

#install(
#  DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  USE_SOURCE_PERMISSIONS
#)

#install(
#  DIRECTORY share/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
#  USE_SOURCE_PERMISSIONS
#)

