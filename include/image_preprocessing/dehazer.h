/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class Dehazer {
 public:
  Dehazer(ros::NodeHandle nh, ros::NodeHandle nhp);
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;
  image_transport::Publisher img_pub_;

  std::string camera_frame_id_;

  cv::Mat dehaze(cv::Mat img);
  cv::Mat boxFilter(cv::Mat img, int local_window_radius);
  cv::Mat guidedFilter(cv::Mat img, cv::Mat guidance_img,
                                int local_window_radius, double eps);
  void imageCallback(
    const sensor_msgs::ImageConstPtr      &image_msg,
    const sensor_msgs::CameraInfoConstPtr &info_msg);
};

