/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <ros/ros.h>
#include <image_preprocessing/mono_dehazer.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "dehazer");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  MonoDehazer d(nh, nhp);

  ros::spin();
  return 0;
}
