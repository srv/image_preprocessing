/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

Mat gain_;
Mat offset_;
Mat avg_;
image_transport::CameraSubscriber img_sub_;
image_transport::CameraPublisher img_pub_;

void imageCallback(const sensor_msgs::ImageConstPtr &image_msg,
                   const sensor_msgs::CameraInfoConstPtr &info_msg) {

  // Exit if no subscribers
  if (img_pub_.getNumSubscribers() == 0) return;

  const cv::Mat img(image_msg->height, image_msg->width, CV_8UC1,
    const_cast<uint8_t*>(&image_msg->data[0]), image_msg->step);

  // Convert image to double
  Mat img_d(img.size(), CV_64FC1);
  img.convertTo(img_d, CV_64F, 1/255.0);

  // Correct
  Mat corrected;
  multiply(img_d - avg_, gain_, corrected);
  corrected = corrected + avg_; // + offset_;

  Mat corrected_uchar;
  corrected = corrected * 255.0;
  corrected.convertTo(corrected_uchar, CV_8U);

  // Publish
  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(image_msg->header,
                                                     image_msg->encoding,
                                                     corrected_uchar).toImageMsg();
  img_pub_.publish(out_msg, info_msg);
}

void vignettingRemover(ros::NodeHandle nh, ros::NodeHandle nhp)  {
  // Read the images
  string mean_filename, std_filename;
  nhp.param("mean_filename", mean_filename, string(""));
  nhp.param("std_filename",  std_filename,  string(""));
  ROS_INFO_STREAM("Reading mean image: " << mean_filename.c_str() <<
                  " and std image: " << std_filename.c_str());
  Mat mean_uchar = imread(mean_filename, CV_LOAD_IMAGE_GRAYSCALE);
  Mat std_uchar  = imread(std_filename,  CV_LOAD_IMAGE_GRAYSCALE);

  // Convert them to double
  Mat mean_d(mean_uchar.size(), CV_64FC1);
  Mat std_d(std_uchar.size(), CV_64FC1);
  mean_uchar.convertTo(mean_d, CV_64F, 1/255.0);
  Scalar uc = mean_uchar.at<unsigned char>(100, 100);
  Scalar db = mean_d.at<double>(100, 100);
  std::cout << "Values: " << uc[0] << " and " << db[0];
  std_uchar.convertTo(std_d, CV_64F, 1/255.0);
  avg_ = mean_d;

  // Compute gain and offset for posterior correction
  Mat desired_avg = 0.05*Mat::ones(mean_d.size(), CV_64FC1);
  double desired_sigma = 1.0/6;
  divide(Mat::ones(std_d.size(), CV_64FC1), std_d, gain_, desired_sigma);
  offset_ = desired_avg - mean_d;
  ROS_INFO_STREAM("Gain and offset ready!");

  // Prepare publisher
  string output_camera_namespace;
  string image_topic = ros::names::remap("image");
  image_transport::ImageTransport it(nh);
  nhp.param("output_namespace", output_camera_namespace, string("vig"));
  img_pub_ = it.advertiseCamera(output_camera_namespace+"/image_raw", 1);

  // Subscribe to images
  ROS_INFO_STREAM("Subscribing to " << image_topic);
  img_sub_ = it.subscribeCamera(image_topic, 1, &imageCallback);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vignetting_remover");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  vignettingRemover(nh, nhp);
  ros::spin();
}
