/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef STEREO_DEHAZER_H
#define STEREO_DEHAZER_H

#include <image_preprocessing/dehazer.h>

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
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_image_proc/processor.h>
#include <boost/thread/mutex.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

class StereoDehazer {
 public:
  StereoDehazer() {}
  StereoDehazer(ros::NodeHandle nh, ros::NodeHandle nhp);

 private:
  // Node handlers
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  Dehazer d_;

  // Topic subscribers
  std::string left_color_topic_, right_color_topic_;
  std::string left_mono_topic_, right_mono_topic_;
  std::string left_info_topic_, right_info_topic_;
  std::string cloud_topic_;

  // Subscribers
  image_transport::SubscriberFilter left_color_sub_,
                                    right_color_sub_,
                                    left_mono_sub_,
                                    right_mono_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_color_info_sub_,
                                                       right_color_info_sub_,
                                                       left_mono_info_sub_,
                                                       right_mono_info_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_color_sub_;

  // Topic sync properties (with pointcloud)
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo,
                                                          sensor_msgs::CameraInfo,
                                                          sensor_msgs::PointCloud2> SyncColorPolicy;
  typedef message_filters::Synchronizer<SyncColorPolicy> SyncColor;
  boost::shared_ptr<SyncColor> sync_color_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo,
                                                          sensor_msgs::CameraInfo> SyncMonoPolicy;
  typedef message_filters::Synchronizer<SyncMonoPolicy> SyncMono;
  boost::shared_ptr<SyncMono> sync_mono_;

  // ROS Camera publisher
  //image_transport::CameraPublisher left_color_pub_, right_color_pub_;
  image_transport::CameraPublisher left_mono_pub_, right_mono_pub_, left_color_pub_, right_color_pub_;

  // points2 publisher
  ros::Publisher pub_points2_;


  void colorCb(const sensor_msgs::ImageConstPtr& l_img_msg,
               const sensor_msgs::ImageConstPtr& r_img_msg,
               const sensor_msgs::CameraInfoConstPtr& l_info_msg,
               const sensor_msgs::CameraInfoConstPtr& r_info_msg,
               const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void monoCb(const sensor_msgs::ImageConstPtr& l_img_msg,
              const sensor_msgs::ImageConstPtr& r_img_msg,
              const sensor_msgs::CameraInfoConstPtr& l_info_msg,
              const sensor_msgs::CameraInfoConstPtr& r_info_msg);
};
#endif
