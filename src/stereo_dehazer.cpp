/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <image_preprocessing/stereo_dehazer.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;

StereoDehazer::StereoDehazer(ros::NodeHandle nh, ros::NodeHandle nhp)
    : nh_(nh), nhp_(nhp) {
  // Topic parameters
  string stereo_ns = nh_.resolveName("stereo");
  nhp_.param("left_mono_topic", left_mono_topic_,
    string("/scaled/left/image_rect"));
  nhp_.param("right_mono_topic", right_mono_topic_,
    string("/scaled/right/image_rect"));
  nhp_.param("left_mono_info_topic", left_mono_info_topic_,
    string("/scaled/left/camera_info"));
  nhp_.param("right_mono_info_topic", right_mono_info_topic_,
    string("/scaled/right/camera_info"));
  nhp_.param("left_color_topic", left_color_topic_,
    string("/left/image_rect_color"));
  nhp_.param("right_color_topic", right_color_topic_,
    string("/right/image_rect_color"));
  nhp_.param("left_info_topic", left_info_topic_,
    string("/left/camera_info"));
  nhp_.param("right_info_topic", right_info_topic_,
    string("/right/camera_info"));

  block_matcher_.setDisparityRange(96);
  block_matcher_.setPreFilterSize(9);
  block_matcher_.setPreFilterCap(31);
  block_matcher_.setCorrelationWindowSize(17);
  block_matcher_.setMinDisparity(2);
  block_matcher_.setTextureThreshold(10);
  block_matcher_.setUniquenessRatio(10);
  block_matcher_.setSpeckleSize(200);
  block_matcher_.setSpeckleRange(4);

  // Topics subscriptions
  image_transport::ImageTransport it(nh_);
  left_mono_sub_.subscribe(it,
    ros::names::clean(stereo_ns + left_mono_topic_),  1);
  right_mono_sub_.subscribe(it,
    ros::names::clean(stereo_ns + right_mono_topic_), 1);
  left_mono_info_sub_.subscribe(nh_,
    ros::names::clean(stereo_ns + left_info_topic_),  1);
  right_mono_info_sub_.subscribe(nh_,
    ros::names::clean(stereo_ns + right_info_topic_), 1);

  sync_pc_.reset(new SyncStereo(StereoSyncPolicy(2),
                                  left_color_sub_,
                                  right_color_sub_,
                                  left_info_sub_,
                                  right_info_sub_) );
  sync_pc_->registerCallback(boost::bind(
      &StereoDehazer::points2Cb,
      this, _1, _2, _3, _4));

  sync_mono_.reset(new SyncStereo(StereoSyncPolicy(2),
                                  left_mono_sub_,
                                  right_mono_sub_,
                                  left_mono_info_sub_,
                                  right_mono_info_sub_) );
  sync_mono_->registerCallback(boost::bind(
      &StereoDehazer::callbackMono,
      this, _1, _2, _3, _4));

  // Set the image publishers before the streaming
  left_mono_pub_   = it.advertiseCamera(
    ros::names::clean(stereo_ns + "/scaled/enhanced/left/image_rect"),  1);
  right_mono_pub_  = it.advertiseCamera(
    ros::names::clean(stereo_ns + "/scaled/enhanced/right/image_rect"), 1);

  // Create the callback with the clouds
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_points2_  = nh.advertise<PointCloud2>(
    ros::names::clean(stereo_ns + "/enhanced/points2"),
    1, boost::bind(&StereoDehazer::connectCb, this),
    boost::bind(&StereoDehazer::connectCb, this));

  ROS_INFO("Stereo Image Enhacer Initialized!");
}

// Handles (un)subscribing when clients (un)subscribe
void StereoDehazer::connectCb() {
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_points2_.getNumSubscribers() == 0) {
    ROS_INFO("[SIE]: Unsubscribing from left & disparity images.");
    left_color_sub_.unsubscribe();
    right_color_sub_ .unsubscribe();
    left_info_sub_ .unsubscribe();
    right_info_sub_.unsubscribe();
  } else if (!left_color_sub_.getSubscriber()) {
    ROS_INFO("[SIE]: Subscribing to left & disparity images.");
    image_transport::ImageTransport it(nh_);
    string stereo_ns = nh_.resolveName("stereo");
    left_color_sub_ .subscribe(it,
      ros::names::clean(stereo_ns + left_color_topic_), 1);
    right_color_sub_  .subscribe(it,
      ros::names::clean(stereo_ns + right_color_topic_),  1);
    left_info_sub_  .subscribe(nh_,
      ros::names::clean(stereo_ns + left_info_topic_),  1);
    right_info_sub_ .subscribe(nh_,
      ros::names::clean(stereo_ns + right_info_topic_), 1);
  }
}

/** \brief Stereo callback. This function is called when synchronized
  * mono image pairs are received.
  * @return
  * \param l_img left stereo image message of type sensor_msgs::Image
  * \param r_img right stereo image message of type sensor_msgs::Image
  * \param l_info left stereo info message of type sensor_msgs::CameraInfo
  * \param r_info right stereo info message of type sensor_msgs::CameraInfo
  */
void StereoDehazer::callbackMono(const ImageConstPtr& l_img_msg,
                                 const ImageConstPtr& r_img_msg,
                                 const CameraInfoConstPtr& l_info_msg,
                                 const CameraInfoConstPtr& r_info_msg) {
  if (left_mono_pub_.getNumSubscribers() > 0) {
    Mat l_img_mono = cv_bridge::toCvShare(l_img_msg,
                                        image_encodings::MONO8)->image;
    Mat deh_mono_left  = d_.dehazeMono(l_img_mono);
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
    Mat deh_mono_right  = d_.dehazeMono(r_img_mono);
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

PointCloud2Ptr StereoDehazer::getPointcloud(
    const ImageConstPtr& l_image_msg,
    const stereo_msgs::DisparityImageConstPtr& disp_msg) {
  // Calculate point cloud
  const Image& dimage = disp_msg->image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  model_.projectDisparityImageTo3d(dmat, points_mat_, true);
  cv::Mat_<cv::Vec3f> mat = points_mat_;

  // Fill in new PointCloud2 message (2D image-like layout)
  PointCloud2Ptr points_msg = boost::make_shared<PointCloud2>();
  points_msg->header = disp_msg->header;
  points_msg->height = mat.rows;
  points_msg->width  = mat.cols;
  points_msg->fields.resize (4);
  points_msg->fields[0].name = "x";
  points_msg->fields[0].offset = 0;
  points_msg->fields[0].count = 1;
  points_msg->fields[0].datatype = PointField::FLOAT32;
  points_msg->fields[1].name = "y";
  points_msg->fields[1].offset = 4;
  points_msg->fields[1].count = 1;
  points_msg->fields[1].datatype = PointField::FLOAT32;
  points_msg->fields[2].name = "z";
  points_msg->fields[2].offset = 8;
  points_msg->fields[2].count = 1;
  points_msg->fields[2].datatype = PointField::FLOAT32;
  points_msg->fields[3].name = "rgb";
  points_msg->fields[3].offset = 12;
  points_msg->fields[3].count = 1;
  points_msg->fields[3].datatype = PointField::FLOAT32;
  // points_msg->is_bigendian = false; ???
  static const int STEP = 16;
  points_msg->point_step = STEP;
  points_msg->row_step = points_msg->point_step * points_msg->width;
  points_msg->data.resize(points_msg->row_step * points_msg->height);
  points_msg->is_dense = false;  // there may be invalid points

  float bad_point = std::numeric_limits<float>::quiet_NaN();
  int offset = 0;
  for (int v = 0; v < mat.rows; ++v) {
    for (int u = 0; u < mat.cols; ++u, offset += STEP) {
      if (isValidPoint(mat(v, u))) {
        // x,y,z,rgba
        memcpy(&points_msg->data[offset + 0], &mat(v, u)[0], sizeof(float));
        memcpy(&points_msg->data[offset + 4], &mat(v, u)[1], sizeof(float));
        memcpy(&points_msg->data[offset + 8], &mat(v, u)[2], sizeof(float));
      } else {
        memcpy(&points_msg->data[offset + 0], &bad_point, sizeof(float));
        memcpy(&points_msg->data[offset + 4], &bad_point, sizeof(float));
        memcpy(&points_msg->data[offset + 8], &bad_point, sizeof(float));
      }
    }
  }

  // Fill in color
  namespace enc = sensor_msgs::image_encodings;
  const std::string& encoding = l_image_msg->encoding;
  offset = 0;
  if (encoding == enc::RGB8) {
    const cv::Mat_<cv::Vec3b> color(l_image_msg->height, l_image_msg->width,
                                    (cv::Vec3b*)&l_image_msg->data[0],
                                    l_image_msg->step);
    // Process the image
    cv::Mat color_dehazed  = d_.dehaze(color);
    for (int v = 0; v < mat.rows; ++v) {
      for (int u = 0; u < mat.cols; ++u, offset += STEP) {
        if (isValidPoint(mat(v, u))) {
          const cv::Vec3b& rgb = color_dehazed.at<cv::Vec3b>(v, u);
          int32_t rgb_packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
          memcpy(&points_msg->data[offset + 12], &rgb_packed, sizeof(int32_t));
        } else {
          memcpy(&points_msg->data[offset + 12], &bad_point, sizeof(float));
        }
      }
    }
  } else if (encoding == enc::BGR8) {
    const cv::Mat_<cv::Vec3b> color(l_image_msg->height, l_image_msg->width,
                                    (cv::Vec3b*)&l_image_msg->data[0],
                                    l_image_msg->step);
    // Process the image
    const cv::Mat_<cv::Vec3b> color_dehazed(d_.dehaze(color));
    for (int v = 0; v < mat.rows; ++v) {
      for (int u = 0; u < mat.cols; ++u, offset += STEP) {
        if (isValidPoint(mat(v, u))) {
          const cv::Vec3b& bgr = color_dehazed.at<cv::Vec3b>(v, u);
          int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
          memcpy(&points_msg->data[offset + 12], &rgb_packed, sizeof(int32_t));
        } else {
          memcpy(&points_msg->data[offset + 12], &bad_point, sizeof(float));
        }
      }
    }
  } else {
    ROS_WARN_THROTTLE(30, "Could not fill color channel of the point cloud, "
                          "unsupported encoding '%s'", encoding.c_str());
  }
  return points_msg;
}

stereo_msgs::DisparityImagePtr StereoDehazer::getDisparity(
    const ImageConstPtr& l_image_msg,
    const ImageConstPtr& r_image_msg,
    const CameraInfoConstPtr& l_info_msg,
    const CameraInfoConstPtr& r_info_msg) {
  // Allocate new disparity image message
  stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage>();
  disp_msg->header         = l_info_msg->header;
  disp_msg->image.header   = l_info_msg->header;

  // Compute window of (potentially) valid disparities
  int border   = block_matcher_.getCorrelationWindowSize() / 2;
  int left   = block_matcher_.getDisparityRange()
    + block_matcher_.getMinDisparity() + border - 1;
  int wtf = (block_matcher_.getMinDisparity() >= 0) ?
    border + block_matcher_.getMinDisparity() :
    std::max(border, -block_matcher_.getMinDisparity());
  int right  = disp_msg->image.width - 1 - wtf;
  int top    = border;
  int bottom = disp_msg->image.height - 1 - border;
  disp_msg->valid_window.x_offset = left;
  disp_msg->valid_window.y_offset = top;
  disp_msg->valid_window.width    = right - left;
  disp_msg->valid_window.height   = bottom - top;

  // Create cv::Mat views onto all buffers
  const cv::Mat_<uint8_t> l_image = cv_bridge::toCvShare(l_image_msg,
    sensor_msgs::image_encodings::MONO8)->image;
  const cv::Mat_<uint8_t> r_image = cv_bridge::toCvShare(r_image_msg,
    sensor_msgs::image_encodings::MONO8)->image;

  // Perform block matching to find the disparities
  block_matcher_.processDisparity(l_image, r_image, model_, *disp_msg);

  // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
  double cx_l = model_.left().cx();
  double cx_r = model_.right().cx();
  if (cx_l != cx_r) {
    cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
                              reinterpret_cast<float*>(&disp_msg->image.data[0]),
                              disp_msg->image.step);
    cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);
  }
  return disp_msg;
}

void StereoDehazer::points2Cb(const ImageConstPtr& l_image_msg,
                              const ImageConstPtr& r_image_msg,
                              const CameraInfoConstPtr& l_info_msg,
                              const CameraInfoConstPtr& r_info_msg) {
  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);
  stereo_msgs::DisparityImagePtr disp_msg = getDisparity(l_image_msg, r_image_msg,
                                            l_info_msg, r_info_msg);
  PointCloud2Ptr points_msg = getPointcloud(l_image_msg, disp_msg);
  pub_points2_.publish(points_msg);
}
