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

// #include <algorithm>

int minimum(int x, int y, int z) {
  int min = x;
  if (y < min) {
    min = y;
  }
  if (z < min) {
    min = z;
  }
  return min;
}

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
  void imageCallback(
    const sensor_msgs::ImageConstPtr      &image_msg,
    const sensor_msgs::CameraInfoConstPtr &info_msg);
};

Dehazer::Dehazer(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp), it_(nh) {
  camera_sub_ = it_.subscribeCamera("image",
                                    1, // queue size
                                    &Dehazer::imageCallback,
                                    this); // transport
  img_pub_ = it_.advertise("dehazed", 1);
}

void Dehazer::imageCallback(
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
  cv::Mat img = cv_image_ptr->image;
  cv::Mat dehazed = dehaze(img);
  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(image_msg->header,
                                                     "bgr8",
                                                     dehazed).toImageMsg();
  img_pub_.publish(out_msg);
}

cv::Mat Dehazer::dehaze(cv::Mat img) {
  ROS_INFO("Dehazing...");
  cv::Mat dark_channel(img.size(), CV_32FC1);
  for (size_t i = 0; i < img.rows; i++) {
    for (size_t j = 0; j < img.cols; j++) {
      cv::Vec3b colour = img.at<cv::Vec3b>(i, j);
      int red = colour[2];
      int green = colour[1];
      int blue = colour[0];
      int val = static_cast<float>(minimum(red, green, blue));
      // ROS_INFO_STREAM("Max is " << val);
      dark_channel.at<float>(i, j) = val;
    }
  }
  // ROS_INFO_STREAM("black ( " << (int)black.rows << ", " << (int)black.cols << ")...");
  double min_dark_channel, max_dark_channel;
  cv::minMaxLoc(dark_channel, &min_dark_channel, &max_dark_channel);
  ROS_INFO_STREAM("max_dark_channel is " << max_dark_channel);
  cv::Mat ones = cv::Mat::ones(img.size(), CV_32FC1);
  cv::divide(dark_channel, max_dark_channel, dark_channel);
  // black = black / max_dark_channel;

  // ROS_INFO_STREAM("ones ( " << (int)ones.rows << ", " << (int)ones.cols << ")...");
  cv::Mat t = ones - 0.6*dark_channel; // if t < 0.1, t = 0.1

  cv::namedWindow("Black", 0);
  cv::imshow("Black", t);
  cv::waitKey(3);

  for (size_t i = 0; i < t.rows; i++) {
    for (size_t j = 0; j < t.cols; j++) {
      float val = t.at<float>(i, j);
      if (val < 0.1)
        t.at<float>(i, j) = 0.1;
    }
  }
  /* First, we have to allocate the new channels */
  cv::Mat r(img.rows, img.cols, CV_8UC1);
  cv::Mat b(img.rows, img.cols, CV_8UC1);
  cv::Mat g(img.rows, img.cols, CV_8UC1);

  /* Now we put these into a matrix */
  cv::Mat out[] = {b, g, r};

  /* Split the image into the three color channels */
  cv::split(img, out);
  r.convertTo(r, CV_32FC1);
  g.convertTo(g, CV_32FC1);
  b.convertTo(b, CV_32FC1);
  r = r - (ones-t)*max_dark_channel;
  g = g - (ones-t)*max_dark_channel;
  b = b - (ones-t)*max_dark_channel;
  cv::divide(r, t, r);
  cv::divide(g, t, g);
  cv::divide(b, t, b);
  r.convertTo(r, CV_8UC1);
  g.convertTo(g, CV_8UC1);
  b.convertTo(b, CV_8UC1);
  cv::Mat dehazed(img.rows, img.cols, CV_8UC3);
  cv::merge(out, 3, dehazed);
  return dehazed;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dehazing");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  Dehazer d(nh, nhp);

  ros::spin();
  return 0;
}
