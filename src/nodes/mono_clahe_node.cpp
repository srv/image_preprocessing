#include <ros/ros.h>

#include "image_preprocessing/mono_clahe.h"


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "clahe");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  MonoClahe node(nh, nhp);

  ros::spin();
  return 0;
}
