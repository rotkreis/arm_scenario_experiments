cmake_minimum_required(VERSION 2.8.3)
project(ann4smc)

find_package(catkin REQUIRED COMPONENTS
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

catkin_package(
  CATKIN_DEPENDS
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

find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(OpenCV 2 REQUIRED)

include_directories(
    include 
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS} 
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

add_executable(image_saver src/image_saver.cpp)
target_link_libraries(image_saver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

