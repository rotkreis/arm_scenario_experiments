#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

boost::format filename_format_;


void saveImage(sensor_msgs::CompressedImage::ConstPtr msg, const int n) //sensor_msgs::Image::ConstPtr msg  || or  ||  sensor_msgs::CompressedImage::ConstPtr msg
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  filename_format_.parse( std::string("/home/masson/Images/frame%04i.jpg") );
  std::string filename = (filename_format_ % n).str();
  cv::imwrite(filename, cv_ptr->image);
}


int main(int argc, char** argv)
{
  rosbag::Bag bag;
  bag.open("/home/masson/Documents/DataSets/baxter/original_data/Thu_May_19_12_33_17_2016/pose1_head_pan.bag", rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("/recorded/cameras/head_camera/image/republished/compressed"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  int c=0;
  foreach(rosbag::MessageInstance const m, view)
  {
      c++;
      std::cout << "/* message 1*/" << c << std::endl;

      sensor_msgs::Image::ConstPtr im = m.instantiate<sensor_msgs::Image>();
      if (im != NULL)
          std::cout << "/* message 2*/" << c << std::endl;


      sensor_msgs::CompressedImage::ConstPtr im2 = m.instantiate<sensor_msgs::CompressedImage>();
      if (im2 != NULL)
          std::cout << "/* message 3*/" << c << std::endl;
          saveImage(im2, c);

  }

  bag.close();
  return 0;
}
