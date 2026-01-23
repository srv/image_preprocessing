#include <ros/ros.h>

#include "image_preprocessing/stereo_clahe.h"


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "stereo_clahe");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  StereoClahe node(nh, nhp);

  ros::spin();
  return 0;
}