/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <image_preprocessing/mono_dehazer.h>
#include <image_preprocessing/dehazer.h>

#include <cv_bridge/cv_bridge.h>

using namespace cv;

MonoDehazer::MonoDehazer(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp), it_(nh) {
  camera_sub_ = it_.subscribeCamera("image",
                                    1, // queue size
                                    &MonoDehazer::imageCallback,
                                    this); // transport
  img_pub_ = it_.advertise("dehazed", 1);
}

void MonoDehazer::imageCallback(
  const sensor_msgs::ImageConstPtr &image_msg,
  const sensor_msgs::CameraInfoConstPtr &info_msg) {
  cv_bridge::CvImagePtr cv_image_ptr;

  try {
    cv_image_ptr = cv_bridge::toCvCopy(image_msg,
                                       sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  Mat img = cv_image_ptr->image;
  Dehazer d;
  Mat dehazed = d.dehaze(img);
  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(image_msg->header,
                                                     "bgr8",
                                                     dehazed).toImageMsg();
  img_pub_.publish(out_msg);
}
