#ifndef IMAGE_PREPROCESSING_MONO_CLAHE_H
#define IMAGE_PREPROCESSING_MONO_CLAHE_H


#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include "image_preprocessing/clahe.h"


class MonoClahe
{
    public:

        MonoClahe(const ros::NodeHandle& nh, 
                  const ros::NodeHandle& nhp);

    protected:

        void imageCallback(const sensor_msgs::ImageConstPtr& img_msg);

    private:

        bool fromImgMsgToMat(const sensor_msgs::ImageConstPtr& img_msg,
                             cv_bridge::CvImagePtr& img_ptr);
    
        ros::NodeHandle nh_;
    
        ros::NodeHandle nhp_;
        
        std::unique_ptr<Clahe> enhancer_;

        image_transport::ImageTransport it_;
        
        image_transport::Subscriber img_sub_;
        
        image_transport::Publisher img_pub_;
};


#endif // IMAGE_PREPROCESSING_MONO_CLAHE_H