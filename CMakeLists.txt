cmake_minimum_required(VERSION 2.8.3)
project(ann4smc)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  std_msgs
  image_transport
  baxter_core_msgs
  baxter_interface
  rosbag
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  roscpp
  sensor_msgs
  std_msgs
  image_transport
  baxter_core_msgs
  baxter_interface
  rosbag
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
  USE_SOURCE_PERMISSIONS
)


