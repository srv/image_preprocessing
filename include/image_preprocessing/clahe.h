#ifndef IMAGE_PREPROCESSING_CLAHE_H
#define IMAGE_PREPROCESSING_CLAHE_H

#include <vector>

#include <opencv2/opencv.hpp>


class Clahe
{
    public:

        Clahe(const double clip_limit);

        cv::Mat correctGrayscale(const cv::Mat& img);

        cv::Mat correctBGR(const cv::Mat& img);

    private:

        double clip_limit_;

        cv::Ptr<cv::CLAHE> clahe_;
};


#endif // IMAGE_PREPROCESSING_CLAHE_H