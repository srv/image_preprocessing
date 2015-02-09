/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <image_preprocessing/stereo_dehazer.h>
#include <image_preprocessing/dehazer.h>

#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;

StereoDehazer::StereoDehazer(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp) {

  // Topic parameters
  string odom_topic, left_topic, right_topic, left_info_topic, right_info_topic, cloud_topic;
  nhp_.param("left_topic",       left_topic,       string("/left/image_color"));
  nhp_.param("right_topic",      right_topic,      string("/right/image_color"));
  nhp_.param("left_info_topic",  left_info_topic,  string("/left/camera_info"));
  nhp_.param("right_info_topic", right_info_topic, string("/right/camera_info"));

  // Topics subscriptions
  image_transport::ImageTransport it(nh_);
  left_sub_       .subscribe(it,  left_topic,       3);
  right_sub_      .subscribe(it,  right_topic,      3);
  left_info_sub_  .subscribe(nh_, left_info_topic,  3);
  right_info_sub_ .subscribe(nh_, right_info_topic, 3);

  // Create the callback with the clouds
  sync_stereo_.reset(new SyncStereo(StereoSyncPolicy(4),
                                  left_sub_,
                                  right_sub_,
                                  left_info_sub_,
                                  right_info_sub_) );
  sync_stereo_->registerCallback(boost::bind(
      &StereoDehazer::callback,
      this, _1, _2, _3, _4));

  // Set the image publishers before the streaming
  left_color_pub_  = it.advertiseCamera("dehazed/left/image_color",  1);
  right_color_pub_ = it.advertiseCamera("dehazed/right/image_color", 1);
  left_mono_pub_   = it.advertiseCamera("dehazed/left/image_mono",  1);
  right_mono_pub_  = it.advertiseCamera("dehazed/right/image_mono", 1);
}


/** \brief Stereo callback. This function is called when synchronized image
  * pairs are received.
  * @return
  * \param l_img left stereo image message of type sensor_msgs::Image
  * \param r_img right stereo image message of type sensor_msgs::Image
  * \param l_info left stereo info message of type sensor_msgs::CameraInfo
  * \param r_info right stereo info message of type sensor_msgs::CameraInfo
  */
void StereoDehazer::callback(const ImageConstPtr& l_img_msg,
                             const ImageConstPtr& r_img_msg,
                             const CameraInfoConstPtr& l_info_msg,
                             const CameraInfoConstPtr& r_info_msg) {
  Mat l_img, r_img;
  try {
    l_img = cv_bridge::toCvCopy(l_img_msg, image_encodings::BGR8)->image;
    r_img = cv_bridge::toCvCopy(r_img_msg, image_encodings::BGR8)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  Dehazer d;
  Mat dehazed_left  = d.dehaze(l_img);
  Mat dehazed_right = d.dehaze(r_img);

  Mat deh_mono_left, deh_mono_right;

  cvtColor(dehazed_left, deh_mono_left, CV_BGR2GRAY);
  cvtColor(dehazed_right, deh_mono_right, CV_BGR2GRAY);

  // convert OpenCV image to ROS message
  cv_bridge::CvImage left_cvi, right_cvi, mono_left_cvi, mono_right_cvi;
  left_cvi.header.stamp = l_img_msg->header.stamp;
  left_cvi.header.frame_id = l_img_msg->header.frame_id;
  left_cvi.encoding = "bgr8";
  left_cvi.image = dehazed_left;

  right_cvi.header.stamp = r_img_msg->header.stamp;
  right_cvi.header.frame_id = r_img_msg->header.frame_id;
  right_cvi.encoding = "bgr8";
  right_cvi.image = dehazed_right;

  mono_left_cvi.header.stamp = l_img_msg->header.stamp;
  mono_left_cvi.header.frame_id = l_img_msg->header.frame_id;
  mono_left_cvi.encoding = "bgr8";
  mono_left_cvi.image = deh_mono_left;

  mono_right_cvi.header.stamp = r_img_msg->header.stamp;
  mono_right_cvi.header.frame_id = r_img_msg->header.frame_id;
  mono_right_cvi.encoding = "bgr8";
  mono_right_cvi.image = deh_mono_right;

  sensor_msgs::Image left_im, right_im, mono_left_im, mono_right_im;
  left_cvi.toImageMsg(left_im);
  right_cvi.toImageMsg(right_im);
  mono_left_cvi.toImageMsg(mono_left_im);
  mono_right_cvi.toImageMsg(mono_right_im);

  left_color_pub_. publish(left_im,  *l_info_msg);
  right_color_pub_.publish(right_im, *r_info_msg);
  left_mono_pub_. publish(mono_left_im,  *l_info_msg);
  right_mono_pub_.publish(mono_right_im, *r_info_msg);
}
