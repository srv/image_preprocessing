/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <image_preprocessing/stereo_dehazer.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;

StereoDehazer::StereoDehazer(ros::NodeHandle nh, ros::NodeHandle nhp)
    : nh_(nh), nhp_(nhp) {
  // Topic parameters
  string stereo_ns = nh_.resolveName("stereo");

  nhp_.param("left_mono_topic", left_mono_topic_,
    string("/left/image_rect"));
  nhp_.param("right_mono_topic", right_mono_topic_,
    string("/right/image_rect"));

  nhp_.param("left_color_topic", left_color_topic_,
    string("/left/image_rect_color"));
  nhp_.param("right_color_topic", right_color_topic_,
    string("/right/image_rect_color"));

  nhp_.param("left_info_topic", left_info_topic_,
    string("/left/camera_info"));
  nhp_.param("right_info_topic", right_info_topic_,
    string("/right/camera_info"));

  nhp_.param("cloud_topic", cloud_topic_,
    string("/points2"));

  // Topics subscriptions
  image_transport::ImageTransport it(nh_);
  left_color_sub_.subscribe(it,
    ros::names::clean(stereo_ns + left_color_topic_),  3);
  right_color_sub_.subscribe(it,
    ros::names::clean(stereo_ns + right_color_topic_), 3);
  left_color_info_sub_.subscribe(nh_,
    ros::names::clean(stereo_ns + left_info_topic_),  3);
  right_color_info_sub_.subscribe(nh_,
    ros::names::clean(stereo_ns + right_info_topic_), 3);
  cloud_color_sub_.subscribe(nh_,
    ros::names::clean(stereo_ns + cloud_topic_), 3);

  left_mono_sub_.subscribe(it,
    ros::names::clean(stereo_ns + left_mono_topic_),  3);
  right_mono_sub_.subscribe(it,
    ros::names::clean(stereo_ns + right_mono_topic_), 3);
  left_mono_info_sub_.subscribe(nh_,
    ros::names::clean(stereo_ns + left_info_topic_),  3);
  right_mono_info_sub_.subscribe(nh_,
    ros::names::clean(stereo_ns + right_info_topic_), 3);

  sync_color_.reset(new SyncColor(SyncColorPolicy(5),
                                  left_color_sub_,
                                  right_color_sub_,
                                  left_color_info_sub_,
                                  right_color_info_sub_,
                                  cloud_color_sub_) );
  sync_color_->registerCallback(boost::bind(
      &StereoDehazer::colorCb,
      this, _1, _2, _3, _4, _5));

  sync_mono_.reset(new SyncMono(SyncMonoPolicy(5),
                                left_mono_sub_,
                                right_mono_sub_,
                                left_mono_info_sub_,
                                right_mono_info_sub_) );
  sync_mono_->registerCallback(boost::bind(
      &StereoDehazer::monoCb,
      this, _1, _2, _3, _4));

  // Set the image publishers before the streaming
  left_mono_pub_   = it.advertiseCamera(
    ros::names::clean(stereo_ns + "/enhanced/left/image_rect"),  1);
  right_mono_pub_  = it.advertiseCamera(
    ros::names::clean(stereo_ns + "/enhanced/right/image_rect"), 1);

  left_color_pub_   = it.advertiseCamera(
    ros::names::clean(stereo_ns + "/enhanced/left/image_rect_color"),  1);
  right_color_pub_  = it.advertiseCamera(
    ros::names::clean(stereo_ns + "/enhanced/right/image_rect_color"), 1);

  // Create the callback with the clouds
  pub_points2_ = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >(
    ros::names::clean(stereo_ns + "/enhanced/points2"), 1);

  ROS_INFO("Stereo Image Enhacer Initialized!");
}

/** \brief Stereo callback. This function is called when synchronized
  * mono image pairs are received.
  * @return
  * \param l_img left stereo image message of type sensor_msgs::Image
  * \param r_img right stereo image message of type sensor_msgs::Image
  * \param l_info left stereo info message of type sensor_msgs::CameraInfo
  * \param r_info right stereo info message of type sensor_msgs::CameraInfo
  */
void StereoDehazer::monoCb(const ImageConstPtr& l_img_msg,
                           const ImageConstPtr& r_img_msg,
                           const CameraInfoConstPtr& l_info_msg,
                           const CameraInfoConstPtr& r_info_msg) {
  if (left_mono_pub_.getNumSubscribers() > 0) {
    Mat l_img_mono = cv_bridge::toCvShare(l_img_msg,
                                        image_encodings::MONO8)->image;
    Mat deh_mono_left  = d_.dehazeGrayscale(l_img_mono);
    // convert OpenCV image to ROS message
    cv_bridge::CvImage mono_left_cvi;
    mono_left_cvi.header.stamp = l_img_msg->header.stamp;
    mono_left_cvi.header.frame_id = l_img_msg->header.frame_id;
    mono_left_cvi.encoding = "mono8";
    mono_left_cvi.image = deh_mono_left;
    sensor_msgs::Image mono_left_im;
    mono_left_cvi.toImageMsg(mono_left_im);
    left_mono_pub_.publish(mono_left_im,  *l_info_msg);
  }
  if (right_mono_pub_.getNumSubscribers() > 0) {
    Mat r_img_mono = cv_bridge::toCvShare(r_img_msg,
                                        image_encodings::MONO8)->image;
    Mat deh_mono_right  = d_.dehazeGrayscale(r_img_mono);
    // convert OpenCV image to ROS message
    cv_bridge::CvImage mono_right_cvi;
    mono_right_cvi.header.stamp = r_img_msg->header.stamp;
    mono_right_cvi.header.frame_id = r_img_msg->header.frame_id;
    mono_right_cvi.encoding = "mono8";
    mono_right_cvi.image = deh_mono_right;
    sensor_msgs::Image mono_right_im;
    mono_right_cvi.toImageMsg(mono_right_im);
    right_mono_pub_.publish(mono_right_im, *r_info_msg);
  }
}


void StereoDehazer::colorCb(const ImageConstPtr& l_img_msg,
                            const ImageConstPtr& r_img_msg,
                            const CameraInfoConstPtr& l_info_msg,
                            const CameraInfoConstPtr& r_info_msg,
                            const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {

  // Left image must be dehazed if exists a subscription to points2
  Mat l_img_color;
  Mat deh_color_left;
  if (left_color_pub_.getNumSubscribers() > 0 || pub_points2_.getNumSubscribers() > 0) {
    l_img_color = cv_bridge::toCvShare(l_img_msg,
                                        image_encodings::BGR8)->image;
    deh_color_left  = d_.dehazeRGB(l_img_color);
    // convert OpenCV image to ROS message
    cv_bridge::CvImage color_left_cvi;
    color_left_cvi.header.stamp = l_img_msg->header.stamp;
    color_left_cvi.header.frame_id = l_img_msg->header.frame_id;
    color_left_cvi.encoding = "bgr8";
    color_left_cvi.image = deh_color_left;
    sensor_msgs::Image color_left_im;
    color_left_cvi.toImageMsg(color_left_im);
    left_color_pub_.publish(color_left_im,  *l_info_msg);
  }
  if (right_color_pub_.getNumSubscribers() > 0) {
    Mat r_img_color = cv_bridge::toCvShare(r_img_msg,
                                        image_encodings::BGR8)->image;
    Mat deh_color_right  = d_.dehazeRGB(r_img_color);
    // convert OpenCV image to ROS message
    cv_bridge::CvImage color_right_cvi;
    color_right_cvi.header.stamp = r_img_msg->header.stamp;
    color_right_cvi.header.frame_id = r_img_msg->header.frame_id;
    color_right_cvi.encoding = "bgr8";
    color_right_cvi.image = deh_color_right;
    sensor_msgs::Image color_right_im;
    color_right_cvi.toImageMsg(color_right_im);
    right_color_pub_.publish(color_right_im, *r_info_msg);
  }

  // Update the camera model
  if (pub_points2_.getNumSubscribers() > 0) {
    image_geometry::PinholeCameraModel left_cam;
    left_cam.fromCameraInfo(l_info_msg);

    // Split enhanced image in channels
    std::vector<Mat> channels(3);
    split(deh_color_left, channels);
    Mat r, g, b;
    channels[0].convertTo(b,CV_8U);
    channels[1].convertTo(g,CV_8U);
    channels[2].convertTo(r,CV_8U);

    std::vector<Mat> channels2(3);
    split(l_img_color, channels2);
    Mat ro, go, bo;
    channels2[0].convertTo(bo,CV_8U);
    channels2[1].convertTo(go,CV_8U);
    channels2[2].convertTo(ro,CV_8U);

    //  Convert the points2 to pcl
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    fromROSMsg(*cloud_msg, *cloud);

    // For every point into the pointcloud, get the dehazed color from left image
    int32_t tmp_rgb;
    for (uint i=0; i<cloud->size(); i++) {
      pcl::PointXYZRGB p = cloud->points[i];
      if (isfinite(p.x) && isfinite(p.y) && isfinite(p.z)) {
        cv::Point3d xyz(p.x, p.y, p.z);
        cv::Point2d pixel = left_cam.project3dToPixel(xyz);

        // Get the color of the corresponding pixel
        uint8_t pr = r.at<uint8_t>(pixel.y, pixel.x);
        uint8_t pg = g.at<uint8_t>(pixel.y, pixel.x);
        uint8_t pb = b.at<uint8_t>(pixel.y, pixel.x);

        int32_t tmp_rgb = (pr << 16) | (pg << 8) | pb;
        p.rgb = *reinterpret_cast<float*>(&tmp_rgb);

        cloud_out->push_back(p);
      }
    }

    // Republish the pointcloud
    cloud_out->header = pcl_conversions::toPCL(cloud_msg->header);
    pub_points2_.publish(cloud_out);
  }
}
