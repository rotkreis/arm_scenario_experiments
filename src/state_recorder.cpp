#include <ros/ros.h>
#include <vector>
#include <iostream>
#include "rosbag/bag.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/JointState.h"
#include <image_transport/image_transport.h>


rosbag::Bag bag;
ros::Time my_time_1, my_time_2, my_time_3;


void head_camera_callback(const sensor_msgs::ImageConstPtr& my_image ){
    my_time_1 = my_image->header.stamp;
    bag.write("/cameras/head_camera/image_recorded", my_time_1, my_image);
}

void right_hand_camera_callback(const sensor_msgs::ImageConstPtr& my_image ){
    my_time_3 = my_image->header.stamp;
    std::cout << "yop!" << std::endl;
    bag.write("/cameras/right_hand_camera/image_recorded", my_time_3, my_image);
}

void joints_callback(const sensor_msgs::JointStateConstPtr& my_state ){
    my_time_2 = my_state->header.stamp;
    bag.write("/robot/joint_states_recorded", my_time_2, my_state);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "bag_recorder");
  ros::NodeHandle node;
  bag.open("record.bag", rosbag::bagmode::Write);
  ros::Rate rate(10.0);

  image_transport::ImageTransport it(node);

  /* fonctionne mais enregistre les images brutes
  ros::param::set("~image_transport", "compressed");
  std::string s;
  ros::param::get("~image_transport", s);
  std::cout << s << std::endl;
  image_transport::Subscriber sub1 = it.subscribe("/cameras/head_camera/image", 1, head_camera_callback);
  */

  // fonctionne mais enregistre les images brutes
  //image_transport::Subscriber sub1 = it.subscribe("/cameras/head_camera/image", 1, head_camera_callback,  image_transport::TransportHints("compressed"));

  // provoque une erreur de comilation
  //ros::Subscriber sub1 = node.subscribe<sensor_msgs::CompressedImage>("/cameras/head_camera/image/compressed", 1, head_camera_callback);

  // ne recupere pas les messages du topic
  //ros::Subscriber sub1 = node.subscribe<sensor_msgs::Image>("/cameras/head_camera/image/compressed", 1, head_camera_callback);

  // fonctionne mais enregistre les images brutes
  // ros::Subscriber sub1 = node.subscribe<sensor_msgs::Image>("/cameras/head_camera/image", 1, head_camera_callback);
  ros::Subscriber sub3 = node.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1, joints_callback);

  while (ros::ok())
  {
      ros::spinOnce();
      rate.sleep();
      std::cout << "difference is: " << my_time_1.toSec() - my_time_2.toSec() << std::endl;
  }
  bag.close();
  return 0;
}
