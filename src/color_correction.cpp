/// Copyright 2016 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <image_preprocessing/color_correction.h>

#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

ColorCorrection::ColorCorrection(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp), it_(nh) {
  camera_sub_ = it_.subscribeCamera("image",
                                    1, // queue size
                                    &ColorCorrection::imageCallback,
                                    this); // transport

  string output_namespace;
  nhp_.param("output_namespace", output_namespace, string("/cc"));
  nhp_.param("gamma_r", gamma_r, 1.0);
  nhp_.param("gamma_g", gamma_g, 0.7);
  nhp_.param("gamma_b", gamma_b, 0.7);
  nhp_.param("max_r", max_r, 127);
  nhp_.param("max_b", max_b, 255);
  nhp_.param("max_g", max_g, 255);

  img_pub_ = it_.advertiseCamera(
    ros::names::clean(output_namespace + "/image_rect_color"),  1);
}

cv::Mat ColorCorrection::adjustLevelsAndGamma(const cv::Mat& channel, double gamma, double max_value) {
  cv::Mat output = channel;
  size_t i, j;
  #pragma omp parallel for collapse(2)
  for (i = 0; i < channel.rows; i++) {
    for (j = 0; j < channel.cols; j++) {
      double v = channel.at<unsigned char>(i,j);
      double gain = max_value / 255.0;
      unsigned char ov = static_cast<unsigned char>(gain*pow(v, gamma));
      output.at<unsigned char>(i, j) = ov;
    }
  }
}

void ColorCorrection::imageCallback(
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
  Mat out;
  Mat img = cv_image_ptr->image;

  // Check number of channels
  if (img.channels() != 3) {
    ROS_WARN_STREAM("[PreProcessing]: Invalid number of channels: " << img.channels());
    return;
  }

  // Split the image in channels
  std::vector<cv::Mat> channels(3);
  std::vector<cv::Mat> output_channels(3);
  cv::split(img, channels);

  // Apply gamma correction
  output_channels[0] = adjustLevelsAndGamma(channels[0], gamma_b, max_b);
  output_channels[1] = adjustLevelsAndGamma(channels[1], gamma_g, max_g);
  output_channels[2] = adjustLevelsAndGamma(channels[2], gamma_r, max_r);
  cv::merge(output_channels, out);

  // convert OpenCV image to ROS message
  cv_bridge::CvImage mono_cvi;
  mono_cvi.header.stamp = image_msg->header.stamp;
  mono_cvi.header.frame_id = image_msg->header.frame_id;
  mono_cvi.encoding = "bgr8";
  mono_cvi.image = out;
  sensor_msgs::Image mono_im;
  mono_cvi.toImageMsg(mono_im);
  img_pub_.publish(mono_im, *info_msg);
}
