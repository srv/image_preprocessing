/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <ros/ros.h>
#include <image_preprocessing/color_correction.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "color_correction");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  ColorCorrection d(nh, nhp);

  ros::spin();
  return 0;
}
