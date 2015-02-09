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

  cv::Mat img_float;
  img.convertTo(img_float, CV_32F, 1/255.0);

  std::vector<cv::Mat> channels(3);
  cv::split(img_float, channels);

  cv::Mat qr(img.rows, img.cols, CV_32FC1);
  cv::Mat qg(img.rows, img.cols, CV_32FC1);
  cv::Mat qb(img.rows, img.cols, CV_32FC1);

  qr = guidedFilter(channels[2], channels[2], local_window_radius, eps);
  qg = guidedFilter(channels[1], channels[1], local_window_radius, eps);
  qb = guidedFilter(channels[0], channels[0], local_window_radius, eps);

  cv::Mat deh, deh_float;
  cv::Mat q[] = {qb, qg, qr};
  cv::merge(q, 3, deh_float);

  cv::Mat result = (img_float - deh_float) * 5 + deh_float;
  result.convertTo(deh, CV_8U, 255.0);
  return deh;
}

cv::Mat Dehazer::boxFilter(cv::Mat img, int local_window_radius) {
  cv::Mat filt_img;
  cv::boxFilter(img, filt_img, -1,
    cv::Size(2*local_window_radius+1, 2*local_window_radius+1));
  return filt_img;
}

cv::Mat Dehazer::guidedFilter(cv::Mat img,  // p
                              cv::Mat guidance_img,  // I
                              int local_window_radius,  // r
                              double eps) {
  // [hei, wid] = size(I);
  int h = guidance_img.rows;
  int w = guidance_img.cols;
  int r = local_window_radius;
  cv::Mat temp1;

  // N = boxfilter(ones(hei, wid), r);
  // % the size of each local patch; N=(2r+1)^2 except for boundary pixels.
  cv::Mat N = boxFilter(cv::Mat::ones(h, w, CV_32FC1),
                        local_window_radius);

  // mean_I = boxfilter(I, r) ./ N;
  cv::Mat mean_guidance;
  temp1 = boxFilter(guidance_img, r);
  cv::divide(temp1, N, mean_guidance);
  // mean_p = boxfilter(p, r) ./ N;
  cv::Mat mean_img;
  temp1 = boxFilter(img, r);
  cv::divide(temp1, N, mean_img);
  // mean_Ip = boxfilter(I.*p, r) ./ N;
  cv::Mat mean_product;
  temp1 = boxFilter(guidance_img.mul(img), r);
  cv::divide(temp1, N, mean_product);
  // cov_Ip = mean_Ip - mean_I .* mean_p;
  // % this is the covariance of (I, p) in each local patch.
  cv::Mat cov_product = mean_product - mean_guidance.mul(mean_img);
  // mean_II = boxfilter(I.*I, r) ./ N;
  cv::Mat mean_guidance_sqr;
  temp1 = boxFilter(guidance_img.mul(guidance_img), r);
  cv::divide(temp1, N, mean_guidance_sqr);
  // var_I = mean_II - mean_I .* mean_I;
  cv::Mat var_guidance = mean_guidance_sqr - mean_guidance.mul(mean_guidance);

  // a = cov_Ip ./ (var_I + eps); % Eqn. (5) in the paper;
  cv::Mat a = cov_product / (var_guidance + eps);
  // b = mean_p - a .* mean_I; % Eqn. (6) in the paper;
  cv::Mat b = mean_img - a.mul(mean_guidance);

  // mean_a = boxfilter(a, r) ./ N;
  cv::Mat mean_a;
  temp1 = boxFilter(a, r) / N;
  cv::divide(temp1, N, mean_a);
  // mean_b = boxfilter(b, r) ./ N;
  cv::Mat mean_b;
  temp1 = boxFilter(b, r) / N;
  cv::divide(temp1, N, mean_b);

  // q = mean_a .* I + mean_b; % Eqn. (8) in the paper;
  cv::Mat q = mean_a.mul(guidance_img) + mean_b;
  return q;
}
