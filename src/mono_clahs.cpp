/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <image_preprocessing/mono_clahs.h>

#include <cv_bridge/cv_bridge.h>

using namespace cv;

MonoClahs::MonoClahs(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp), it_(nh) {
  camera_sub_ = it_.subscribeCamera("image",
                                    1, // queue size
                                    &MonoClahs::imageCallback,
                                    this); // transport
  img_pub_ = it_.advertise("clahs", 1);
}

void MonoClahs::imageCallback(
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

  // Extract RGB
  Mat img = cv_image_ptr->image;
  std::vector<Mat> channels(3);
  split(img, channels);
  Mat r, g, b;
  channels[2].convertTo(r,CV_8U);
  channels[1].convertTo(g,CV_8U);
  channels[0].convertTo(b,CV_8U);

  // TODO: in separate threads
  Clahs c;
  Mat cr = c.compute(r,5,4);
  Mat cg = c.compute(g,5,4);
  Mat cb = c.compute(b,5,4);

  Mat out;
  Mat q[] = {cb, cg, cr};
  merge(q, 3, out);

  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(image_msg->header,
                                                     "bgr8",
                                                     out).toImageMsg();
  img_pub_.publish(out_msg);
}
