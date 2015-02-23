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
  std::string left_mono_info_topic_, right_mono_info_topic_;
  std::string left_info_topic_, right_info_topic_;

  image_geometry::StereoCameraModel model_;
  cv::Mat_<cv::Vec3f> points_mat_;  // scratch buffer
  stereo_image_proc::StereoProcessor block_matcher_;

  // Subscribers
  image_transport::SubscriberFilter left_color_sub_, right_color_sub_;
  image_transport::SubscriberFilter left_mono_sub_, right_mono_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_,
                                                       right_info_sub_,
                                                       left_mono_info_sub_,
                                                       right_mono_info_sub_;

  // Topic sync properties (with pointcloud)
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::CameraInfo,
                                                    sensor_msgs::CameraInfo>
                                                    StereoSyncPolicy;
  typedef message_filters::Synchronizer<StereoSyncPolicy> SyncStereo;
  boost::shared_ptr<SyncStereo> sync_pc_, sync_mono_;

  // ROS Camera publisher
  //image_transport::CameraPublisher left_color_pub_, right_color_pub_;
  image_transport::CameraPublisher left_mono_pub_, right_mono_pub_;

  // points2 publisher
  boost::mutex connect_mutex_;
  ros::Publisher pub_points2_;

  // Protected functions and callbacks
  void connectCb();

  void callbackMono(const sensor_msgs::ImageConstPtr& l_img_msg,
                    const sensor_msgs::ImageConstPtr& r_img_msg,
                    const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                    const sensor_msgs::CameraInfoConstPtr& r_info_msg);
  void points2Cb(const sensor_msgs::ImageConstPtr& l_img_msg,
                 const sensor_msgs::ImageConstPtr& r_img_msg,
                 const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                 const sensor_msgs::CameraInfoConstPtr& r_info_msg);

  stereo_msgs::DisparityImagePtr getDisparity(
    const sensor_msgs::ImageConstPtr& l_image_msg,
    const sensor_msgs::ImageConstPtr& r_image_msg,
    const sensor_msgs::CameraInfoConstPtr& l_info_msg,
    const sensor_msgs::CameraInfoConstPtr& r_info_msg);

  sensor_msgs::PointCloud2Ptr getPointcloud(
    const sensor_msgs::ImageConstPtr& l_image_msg,
    const stereo_msgs::DisparityImageConstPtr& disp_msg);

  inline bool isValidPoint(const cv::Vec3f& pt) {
    // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
    // and zero disparities (point mapped to infinity).
    return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
  }
};
#endif
