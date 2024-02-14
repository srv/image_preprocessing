/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <thread>

using namespace std;
using namespace cv;


image_transport::Subscriber img_sub_;
image_transport::Publisher img_pub_;

void cv_clahe(cv::Mat _src, cv::Mat& _dst) {
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(2);
  clahe->apply(_src, _dst);
  return;
}

void imageCallback(const sensor_msgs::ImageConstPtr &image_msg) {

  // Exit if no subscribers
  if (img_pub_.getNumSubscribers() == 0) return;

  cv_bridge::CvImagePtr img_ptr;
  try {
    img_ptr = cv_bridge::toCvCopy(image_msg,
                                  sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("[ColorCorrectionImage:] cv_bridge exception: %s", e.what());
  }
  cv::Mat img = img_ptr->image;

  // Split channels
  std::vector<cv::Mat> BGR;
  cv::split(img, BGR);

  // Equalizes the histogram of a one channel image (8UC1) using Contrast
  // Limited Adaptive Histogram Equalization.
  std::thread thread_b(cv_clahe, BGR[0], std::ref(BGR[0]));
  std::thread thread_g(cv_clahe, BGR[1], std::ref(BGR[1]));
  std::thread thread_r(cv_clahe, BGR[2], std::ref(BGR[2]));
  thread_b.join();
  thread_g.join();
  thread_r.join();

  cv::Mat img_out;
  cv::merge(BGR, img_out);

  img_ptr->image = img_out;
  img_pub_.publish(img_ptr->toImageMsg());
}

void colorCorrection(ros::NodeHandle nh, ros::NodeHandle nhp)  {
  // Prepare publisher
  image_transport::ImageTransport it(nh);
  img_pub_ = it.advertise("/color_correction_image/output", 1);

  // Subscribe to images
  img_sub_ = it.subscribe("/color_correction_image/input", 1, &imageCallback);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "color_correction_image");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  colorCorrection(nh, nhp);
  ros::spin();
}
