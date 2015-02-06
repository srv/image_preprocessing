/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <image_preprocessing/dehazer.h>

Dehazer::Dehazer(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp), it_(nh) {
  camera_sub_ = it_.subscribeCamera("image",
                                    1, // queue size
                                    &Dehazer::imageCallback,
                                    this); // transport
  img_pub_ = it_.advertise("dehazed", 1);
}

void Dehazer::imageCallback(
  const sensor_msgs::ImageConstPtr &image_msg,
  const sensor_msgs::CameraInfoConstPtr &info_msg) {
  cv_bridge::CvImagePtr cv_image_ptr;

  try {
    cv_image_ptr = cv_bridge::toCvCopy(image_msg,
                                       sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat img = cv_image_ptr->image;
  cv::Mat dehazed = dehaze(img);
  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(image_msg->header,
                                                     "bgr8",
                                                     dehazed).toImageMsg();
  img_pub_.publish(out_msg);
}

cv::Mat Dehazer::dehaze(cv::Mat img) {
  int local_window_radius = 16;
  double eps = 0.01;

  cv::Mat r(img.rows, img.cols, CV_8UC1);
  cv::Mat b(img.rows, img.cols, CV_8UC1);
  cv::Mat g(img.rows, img.cols, CV_8UC1);
  r.convertTo(r, CV_32FC1);
  g.convertTo(g, CV_32FC1);
  b.convertTo(b, CV_32FC1);

  cv::Mat qr(img.rows, img.cols, CV_32FC1);
  cv::Mat qg(img.rows, img.cols, CV_32FC1);
  cv::Mat qb(img.rows, img.cols, CV_32FC1);

  qr = guidedFilter(r, r, local_window_radius, eps);
  qg = guidedFilter(g, g, local_window_radius, eps);
  qb = guidedFilter(b, b, local_window_radius, eps);

  cv::Mat deh;
  cv::Mat q[] = {qb, qg, qr};
  cv::merge(q, 3, deh);

  deh.convertTo(deh, CV_8UC3);
  return (img - deh) * 5 + deh;
}

cv::Mat Dehazer::boxFilter(cv::Mat img, int local_window_radius) {
  cv::Mat filt_img;
  cv::boxFilter(img, filt_img, CV_32F, cv::Size(2*local_window_radius+1, 2*local_window_radius+1));
  return filt_img;
}

cv::Mat Dehazer::guidedFilter(cv::Mat img,  // p
                              cv::Mat guidance_img,  // I
                              int local_window_radius,  // r
                              double eps) {
  // [hei, wid] = size(I);
  int img_height = guidance_img.rows;
  int img_width  = guidance_img.cols;
  int r = local_window_radius;

  // N = boxfilter(ones(hei, wid), r);
  // % the size of each local patch; N=(2r+1)^2 except for boundary pixels.
  cv::Mat N = boxFilter(cv::Mat::ones(img_height, img_width, CV_32FC1),
                        local_window_radius);

  // mean_I = boxfilter(I, r) ./ N;
  cv::Mat mean_guidance = boxFilter(guidance_img, r) / N;
  // mean_p = boxfilter(p, r) ./ N;
  cv::Mat mean_img = boxFilter(img, r) / N;
  // mean_Ip = boxfilter(I.*p, r) ./ N;
  cv::Mat mean_product = boxFilter(guidance_img.mul(img), r) / N;
  // cov_Ip = mean_Ip - mean_I .* mean_p;
  // % this is the covariance of (I, p) in each local patch.
  cv::Mat cov_product = mean_product - mean_guidance.mul(mean_img);

  // mean_II = boxfilter(I.*I, r) ./ N;
  cv::Mat mean_guidance_sqr = boxFilter(guidance_img.mul(guidance_img), r) / N;
  // var_I = mean_II - mean_I .* mean_I;
  cv::Mat var_guidance = mean_guidance_sqr - mean_guidance.mul(mean_guidance);

  // a = cov_Ip ./ (var_I + eps); % Eqn. (5) in the paper;
  cv::Mat a = cov_product / (var_guidance + eps);
  // b = mean_p - a .* mean_I; % Eqn. (6) in the paper;
  cv::Mat b = mean_img - a.mul(mean_guidance);

  // mean_a = boxfilter(a, r) ./ N;
  cv::Mat mean_a = boxFilter(a, r) / N;
  // mean_b = boxfilter(b, r) ./ N;
  cv::Mat mean_b = boxFilter(b, r) / N;

  // q = mean_a .* I + mean_b; % Eqn. (8) in the paper;
  cv::Mat q = mean_a.mul(guidance_img) + mean_b;
  return q;
}
