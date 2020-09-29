/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef CLAHS_H
#define CLAHS_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <algorithm>
#include <boost/thread.hpp>

using namespace cv;

class Clahs {
  public:
    Clahs();

    void clahsGrayscale(const Mat& Image, Mat& Out, const int& NrY, const int& NrX);
    void clahsRGB(const Mat& Image, Mat& Out, const int& NrY, const int& NrX);

  protected:
    Mat MakeLUT(const int& NrGreyLevels,
                const int& NrBins);

    Mat ClipHistogram(const Mat& Histogram,
                      const int& NrGreyLevels,
                      const double& ClipLimit);

    Mat ClipHistogramSimple(const Mat& Histogram,
                            const double& ClipLimit);

    Mat MapHistogram(const Mat& Histogram,
                     const int& Min,
                     const int& Max,
                     const int& NrGreyLevels,
                     const int& NrofPixels,
                     const int& heq_type,
                     const double& heq_alpha);

    Mat Interpolate(const Mat& SubRegion,
                    const Mat& MapLU,
                    const Mat& MapRU,
                    const Mat& MapLB,
                    const Mat& MapRB,
                    const int& XSize,
                    const int& YSize,
                    const Mat& LUT);

    Mat buildbyIndices(const Mat& vector,
                       const Mat& indices);

    Mat linspace(const double& startP,
                 const double& Endp,
                 const int& interval);

    Mat cumsum(const Mat& in);
};
#endif
