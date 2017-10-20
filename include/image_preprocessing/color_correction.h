#ifndef COLOR_CORRECTION_H
#define COLOR_CORRECTION_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

class ColorCorrection {
 public:
  ColorCorrection(ros::NodeHandle nh, ros::NodeHandle nhp);
  cv::Mat adjustLevelsAndGamma(const cv::Mat& channel,
                               double gamma,
                               double max_value);
  void imageCallback(const sensor_msgs::ImageConstPtr &image_msg,
                     const sensor_msgs::CameraInfoConstPtr &info_msg);
  double gamma_r, gamma_g, gamma_b;
  int max_r, max_b, max_g;
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;
  image_transport::CameraPublisher img_pub_;
};

#endif // COLOR_CORRECTION_H
