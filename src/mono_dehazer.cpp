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

  string output_namespace;
  nhp_.param("output_namespace", output_namespace, string("/dehazed"));

  img_pub_ = it_.advertiseCamera(
    ros::names::clean(output_namespace + "/image_rect_color"),  1);
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

  // Compute
  Dehazer d;
  Mat out;
  string encoding;
  Mat img = cv_image_ptr->image;
  if (img.channels() == 1) {
    out = d.dehazeGrayscale(img);
    encoding = "mono8";
  }
  else if (img.channels() == 3) {
    out = d.dehazeRGB(img);
    encoding = "bgr8";
  }
  else {
    ROS_WARN_STREAM("[PreProcessing]: Invalid number of channels: " << img.channels());
    return;
  }

  // convert OpenCV image to ROS message
  cv_bridge::CvImage mono_cvi;
  mono_cvi.header.stamp = image_msg->header.stamp;
  mono_cvi.header.frame_id = image_msg->header.frame_id;
  mono_cvi.encoding = encoding;
  mono_cvi.image = out;
  sensor_msgs::Image mono_im;
  mono_cvi.toImageMsg(mono_im);
  img_pub_.publish(mono_im, *info_msg);
}
