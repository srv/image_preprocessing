/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef DEHAZER_H
#define DEHAZER_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <boost/thread.hpp>

class Dehazer {
 public:
  Dehazer() {}
  cv::Mat dehazeRGB(cv::Mat img);
  cv::Mat dehazeGrayscale(cv::Mat img);
  void guidedFilter(cv::Mat img, cv::Mat guidance_img, cv::Mat& out,
                       int local_window_radius, double eps);
};
#endif
