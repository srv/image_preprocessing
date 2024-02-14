#ifndef FLICKER_REMOVER_H
#define FLICKER_REMOVER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace std;

class FlickerRemover
{

  public:
    FlickerRemover(ros::NodeHandle nh, ros::NodeHandle nhp);

    void run();

  protected:

  void msgsCallback(const sensor_msgs::ImageConstPtr& l_img_msg,
                    const sensor_msgs::ImageConstPtr& r_img_msg,
                    const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                    const sensor_msgs::CameraInfoConstPtr& r_info_msg);

  bool imgMsgToMat(sensor_msgs::Image l_img_msg,
                   sensor_msgs::Image r_img_msg,
                   cv::Mat &l_img, cv::Mat &r_img);

  void featureExtraction(cv::Mat img, vector<cv::KeyPoint>& kp, cv::Mat& desc);

  void registerImages();

  void thresholdMatching(const cv::Mat& descriptors1, const cv::Mat& descriptors2,
                         double threshold, const cv::Mat& match_mask, vector<cv::DMatch>& matches);

  void crossCheckFilter(const vector<cv::DMatch>& matches1to2,
                        const vector<cv::DMatch>& matches2to1,
                        vector<cv::DMatch>& checked_matches);

  void crossCheckThresholdMatching(const cv::Mat& descriptors1, const cv::Mat& descriptors2,
                                   double threshold, vector<cv::DMatch>& matches);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    double desc_thresh_;
    double epipolar_thresh_;
    double min_inliers_;

    cv::Mat next_l_, next_r_;
    cv::Mat curr_l_, curr_r_;
    cv::Mat prev_l_, prev_r_;

    int frame_counter_;

    // Topic sync
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                            sensor_msgs::Image,
                                                            sensor_msgs::CameraInfo,
                                                            sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;

};
#endif
