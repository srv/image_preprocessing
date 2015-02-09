/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef STEREO_DEHAZER_H
#define STEREO_DEHAZER_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

class StereoDehazer {
 public:
  StereoDehazer(ros::NodeHandle nh, ros::NodeHandle nhp);

 private:
  // Node handlers
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  // Topic subscribers
  image_transport::SubscriberFilter left_sub_, right_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_,
                                                       right_info_sub_;

  // Topic sync properties (with pointcloud)
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::CameraInfo,
                                                    sensor_msgs::CameraInfo>
                                                    StereoSyncPolicy;
  typedef message_filters::Synchronizer<StereoSyncPolicy> SyncStereo;
  boost::shared_ptr<SyncStereo> sync_stereo_;

  // ROS Camera publisher
  image_transport::CameraPublisher left_color_pub_, left_mono_pub_;
  image_transport::CameraPublisher right_color_pub_, right_mono_pub_;

  // Protected functions and callbacks
  void callback(const sensor_msgs::ImageConstPtr& l_img_msg,
                const sensor_msgs::ImageConstPtr& r_img_msg,
                const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                const sensor_msgs::CameraInfoConstPtr& r_info_msg);
};
#endif
