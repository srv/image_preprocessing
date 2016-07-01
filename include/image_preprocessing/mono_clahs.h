/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef MONO_CLAHS_H
#define MONO_CLAHS_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <image_preprocessing/clahs.h>

class MonoClahs {
 public:
  MonoClahs(ros::NodeHandle nh, ros::NodeHandle nhp);
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;
  image_transport::Publisher img_pub_;

  void imageCallback(const sensor_msgs::ImageConstPtr &image_msg,
                     const sensor_msgs::CameraInfoConstPtr &info_msg);
};
#endif
