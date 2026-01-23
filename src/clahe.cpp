#include "image_preprocessing/clahe.h"


Clahe::Clahe(const double clip_limit): clip_limit_{clip_limit}
{
    clahe_ = cv::createCLAHE();
    clahe_->setClipLimit(clip_limit_);
    clahe_->setTilesGridSize(cv::Size(8, 8));
}


cv::Mat Clahe::correctGrayscale(const cv::Mat& img)
{
    // Sanity check.
    if (img.empty())
    {
        std::cerr << "Image is empty!" << std::endl;
        return cv::Mat();
    }

    // Second sanity check.
    if (img.type() != CV_8UC3 && img.type() != CV_8UC1)
    {
        std::cerr << "Unsupported image type. Only CV_8UC1 and CV_8UC3 are allowed." << std::endl;
        return cv::Mat();
    }

    // Apply clahe.
    cv::Mat proc_img;
    if (img.type() == CV_8UC3)
    {
        cv::Mat g_img;
        cv::cvtColor(img, g_img, cv::COLOR_BGR2GRAY);
        clahe_->apply(g_img, proc_img);
    }
    else 
    {
        clahe_->apply(img, proc_img);
    }

    return proc_img;
}


cv::Mat Clahe::correctBGR(const cv::Mat& img)
{
    // Sanity check.
    if (img.empty())
    {
        std::cerr << "Image is empty!" << std::endl;
        return cv::Mat();
    }

    // Second sanity check.
    if (img.type() != CV_8UC3)
    {
        std::cerr << "Unsupported image type. Only CV_8UC3 are allowed." << std::endl;
        return cv::Mat();
    }

    // From BGR to LAB
    cv::Mat lab_img;
    cv::cvtColor(img, lab_img, cv::COLOR_BGR2Lab);

    // Split in L, A, and B.
    std::vector<cv::Mat> lab_planes;
    cv::split(lab_img, lab_planes);

    // Apply CLAHE to L.
    clahe_->apply(lab_planes[0], lab_planes[0]);

    // Merge channels.
    cv::merge(lab_planes, lab_img);

    // From LAB to BGR.
    cv::Mat proc_img;
    cv::cvtColor(lab_img, proc_img, cv::COLOR_Lab2BGR);

    return proc_img;
}