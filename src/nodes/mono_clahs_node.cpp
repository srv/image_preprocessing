/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <ros/ros.h>
#include <image_preprocessing/mono_clahs.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "clahs");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  MonoClahs c(nh, nhp);

  ros::spin();
  return 0;
}
