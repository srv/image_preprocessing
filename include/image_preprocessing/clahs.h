/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef CLAHS_H
#define CLAHS_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace cv;

class Clahs {
  public:
    Clahs() {}

  protected:
    Mat makeLUT(int min, int max, int nr_bins);
};
#endif
