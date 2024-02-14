/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <ros/ros.h>
#include <image_preprocessing/stereo_dehazer.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "stereo_dehazer");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  StereoDehazer d(nh, nhp);

  ros::spin();
  return 0;
}
