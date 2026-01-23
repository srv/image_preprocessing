#include "image_preprocessing/mono_clahe.h"


MonoClahe::MonoClahe(const ros::NodeHandle& nh, 
                     const ros::NodeHandle& nhp): nh_{nh}, nhp_{nhp}, it_{nh}
{
    // Get params.
    double clip_limit;
    nhp_.param<double>("clip_limit", clip_limit, 2.0);

    // Set enhancer.
    enhancer_ = std::make_unique<Clahe>(clip_limit);

    // Publisher.
    img_pub_ = it_.advertise("img_out", 1);
    
    // Subscriber.
    img_sub_ = it_.subscribe("img_in", 1, &MonoClahe::imageCallback, this);

    ROS_INFO_STREAM("Stereo CLAHE Node started. Clip limit: " << clip_limit);
}


void MonoClahe::imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
    // Convert and enhace.
    cv::Mat proc_img;
    cv_bridge::CvImagePtr img_ptr;
    if (img_msg->encoding == sensor_msgs::image_encodings::BGR8)
    {
        if(!fromImgMsgToMat(img_msg, img_ptr))
            return;
        proc_img = enhancer_->correctBGR(img_ptr->image);
    }
    else if (img_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
        if(!fromImgMsgToMat(img_msg, img_ptr))
            return;
        proc_img = enhancer_->correctGrayscale(img_ptr->image);
    }
    else
    {
        ROS_ERROR_STREAM("[MonoClahe:] Invalid encoding: " << img_msg->encoding << ". Only BGR8 and MONO8 are supported.");
        return;
    }

    // Publish.
    img_ptr->image = proc_img;
    img_pub_.publish(img_ptr->toImageMsg());
}


bool MonoClahe::fromImgMsgToMat(const sensor_msgs::ImageConstPtr& img_msg,
                                cv_bridge::CvImagePtr& img_ptr)
{
    try
    {
        img_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
    }
    catch (const cv_bridge::Exception& e) 
    {
        ROS_ERROR_STREAM("[MonoClahe:] cv_bridge exception: " << e.what());
        return false;
    }
    return true;
}