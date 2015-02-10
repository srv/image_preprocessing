/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef DEHAZER_H
#define DEHAZER_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class Dehazer {
 public:
  Dehazer() {}
  cv::Mat dehaze(cv::Mat img);
  cv::Mat dehazeMono(cv::Mat img);
  cv::Mat guidedFilter(cv::Mat img, cv::Mat guidance_img,
                       int local_window_radius, double eps);
};
#endif
