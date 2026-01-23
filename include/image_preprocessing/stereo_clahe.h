#ifndef IMAGE_PREPROCESSING_STEREO_CLAHE_H
#define IMAGE_PREPROCESSING_STEREO_CLAHE_H


#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include "image_preprocessing/clahe.h"


class StereoClahe
{
    public:

        StereoClahe(const ros::NodeHandle& nh, 
                    const ros::NodeHandle& nhp);

    protected:

        void stereoCallback(const sensor_msgs::ImageConstPtr& l_img_msg,
                            const sensor_msgs::ImageConstPtr& r_img_msg,
                            const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                            const sensor_msgs::CameraInfoConstPtr& r_info_msg);

    private:

        bool processImg(const sensor_msgs::ImageConstPtr& img_msg, 
                        cv_bridge::CvImagePtr& proc_img_ptr);

        bool fromImgMsgToMat(const sensor_msgs::ImageConstPtr& img_msg,
                             cv_bridge::CvImagePtr& img_ptr);
    
        ros::NodeHandle nh_;
    
        ros::NodeHandle nhp_;
        
        std::unique_ptr<Clahe> enhancer_;

        image_transport::ImageTransport it_;
        
        image_transport::SubscriberFilter l_img_sub_, r_img_sub_;
        
        image_transport::CameraPublisher l_cam_pub_, r_cam_pub_;

        message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                sensor_msgs::Image,
                                                                sensor_msgs::CameraInfo,
                                                                sensor_msgs::CameraInfo> SyncPolicy_;

        typedef message_filters::Synchronizer<SyncPolicy_> Sync_;
        
        std::shared_ptr<Sync_> sync_;
};


#endif // IMAGE_PREPROCESSING_STEREO_CLAHE_H