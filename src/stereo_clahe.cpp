#include "image_preprocessing/stereo_clahe.h"


StereoClahe::StereoClahe(const ros::NodeHandle& nh, 
                         const ros::NodeHandle& nhp): nh_{nh}, nhp_{nhp}, it_{nh}
{
    // Get params.
    double clip_limit;
    nhp_.param<double>("clip_limit", clip_limit, 2.0);

    // Set enhancer.
    enhancer_ = std::make_unique<Clahe>(clip_limit);

    // Publishers.
    l_cam_pub_ = it_.advertiseCamera("l_img_out", 1);
    r_cam_pub_ = it_.advertiseCamera("r_img_out", 1);

    // Subscribers.
    l_img_sub_.subscribe(it_, "l_img_in", 5);
    r_img_sub_.subscribe(it_, "r_img_in", 5);
    l_info_sub_.subscribe(nh_, "l_info_in", 5);
    r_info_sub_.subscribe(nh_, "r_info_in", 5);

    // Message sync.
    sync_ = std::make_shared<Sync_>(SyncPolicy_(10), l_img_sub_, r_img_sub_, l_info_sub_, r_info_sub_);
    sync_->registerCallback(std::bind(&StereoClahe::stereoCallback, this, 
                                      std::placeholders::_1, std::placeholders::_2,
                                      std::placeholders::_3, std::placeholders::_4));

    ROS_INFO_STREAM("Stereo CLAHE Node started. Clip limit: " << clip_limit);
}


void StereoClahe::stereoCallback(const sensor_msgs::ImageConstPtr& l_img_msg,
                                 const sensor_msgs::ImageConstPtr& r_img_msg,
                                 const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                 const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
    if (l_cam_pub_.getNumSubscribers() == 0 && r_cam_pub_.getNumSubscribers() == 0)
        return;

    cv_bridge::CvImagePtr l_proc_img_pr, r_proc_img_pr;
    if (!processImg(l_img_msg, l_proc_img_pr))
        return;
    if (!processImg(r_img_msg, r_proc_img_pr))
        return;

    if (l_cam_pub_.getNumSubscribers() > 0)
        l_cam_pub_.publish(l_proc_img_pr->toImageMsg(), l_info_msg);
    if (r_cam_pub_.getNumSubscribers() > 0)
        r_cam_pub_.publish(r_proc_img_pr->toImageMsg(), r_info_msg);
}


bool StereoClahe::processImg(const sensor_msgs::ImageConstPtr& img_msg,
                             cv_bridge::CvImagePtr& proc_img_ptr)
{
    // Convert and enhace.
    cv::Mat proc_img;
    if (img_msg->encoding == sensor_msgs::image_encodings::BGR8)
    {
        if(!fromImgMsgToMat(img_msg, proc_img_ptr))
            return false;
        proc_img = enhancer_->correctBGR(proc_img_ptr->image);
    }
    else if (img_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
        if(!fromImgMsgToMat(img_msg, proc_img_ptr))
            return false;
        proc_img = enhancer_->correctGrayscale(proc_img_ptr->image);
    }
    else
    {
        ROS_ERROR_STREAM("[StereoClahe:] Invalid encoding: " << img_msg->encoding << ". Only BGR8 and MONO8 are supported.");
        return false;
    }
    proc_img_ptr->image = proc_img;

    return true;
}


bool StereoClahe::fromImgMsgToMat(const sensor_msgs::ImageConstPtr& img_msg,
                                  cv_bridge::CvImagePtr& img_ptr)
{
    try
    {
        img_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
    }
    catch (const cv_bridge::Exception& e) 
    {
        ROS_ERROR_STREAM("[StereoClahe:] cv_bridge exception: " << e.what());
        return false;
    }
    return true;
}