/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <image_preprocessing/dehazer.h>

using namespace cv;

Mat Dehazer::dehaze(Mat img) {
  int local_window_radius = 16;
  double eps = 0.01;

  Mat img_float;
  img.convertTo(img_float, CV_32F, 1/255.0);

  std::vector<Mat> channels(3);
  split(img_float, channels);

  Mat qr(img.rows, img.cols, CV_32FC1);
  Mat qg(img.rows, img.cols, CV_32FC1);
  Mat qb(img.rows, img.cols, CV_32FC1);

  qr = guidedFilter(channels[2], channels[2], local_window_radius, eps);
  qg = guidedFilter(channels[1], channels[1], local_window_radius, eps);
  qb = guidedFilter(channels[0], channels[0], local_window_radius, eps);

  Mat deh, deh_float;
  Mat q[] = {qb, qg, qr};
  merge(q, 3, deh_float);

  Mat result = (img_float - deh_float) * 5 + deh_float;
  result.convertTo(deh, CV_8U, 255.0);
  return deh;
}

Mat Dehazer::dehazeMono(Mat img) {
  int local_window_radius = 16;
  double eps = 0.01;

  Mat img_float;
  img.convertTo(img_float, CV_32F, 1/255.0);

  Mat filt = guidedFilter(img_float, img_float, local_window_radius, eps);

  Mat deh;
  Mat result = (img_float - filt) * 5 + filt;
  result.convertTo(deh, CV_8U, 255.0);
  return deh;
}

Mat Dehazer::guidedFilter(Mat img,  // p
                              Mat guidance_img,  // I
                              int local_window_radius,  // r
                              double eps) {
  // [hei, wid] = size(I);
  int h = guidance_img.rows;
  int w = guidance_img.cols;
  Mat temp1;

  Size size_r(2*local_window_radius+1, 2*local_window_radius+1);

  // N = boxfilter(ones(hei, wid), r);
  // % the size of each local patch; N=(2r+1)^2 except for boundary pixels.
  Mat N;
  boxFilter(Mat::ones(h, w, CV_32FC1), N, -1, size_r);

  // mean_I = boxfilter(I, r) ./ N;
  Mat mean_guidance;
  boxFilter(guidance_img, temp1, -1, size_r);
  divide(temp1, N, mean_guidance);
  // mean_p = boxfilter(p, r) ./ N;
  Mat mean_img;
  boxFilter(img, temp1, -1, size_r);
  divide(temp1, N, mean_img);
  // mean_Ip = boxfilter(I.*p, r) ./ N;
  Mat mean_product;
  boxFilter(guidance_img.mul(img), temp1, -1, size_r);
  divide(temp1, N, mean_product);
  // cov_Ip = mean_Ip - mean_I .* mean_p;
  // % this is the covariance of (I, p) in each local patch.
  Mat cov_product = mean_product - mean_guidance.mul(mean_img);
  // mean_II = boxfilter(I.*I, r) ./ N;
  Mat mean_guidance_sqr;
  boxFilter(guidance_img.mul(guidance_img), temp1, -1, size_r);
  divide(temp1, N, mean_guidance_sqr);
  // var_I = mean_II - mean_I .* mean_I;
  Mat var_guidance = mean_guidance_sqr - mean_guidance.mul(mean_guidance);

  // a = cov_Ip ./ (var_I + eps); % Eqn. (5) in the paper;
  Mat a = cov_product / (var_guidance + eps);
  // b = mean_p - a .* mean_I; % Eqn. (6) in the paper;
  Mat b = mean_img - a.mul(mean_guidance);

  // mean_a = boxfilter(a, r) ./ N;
  Mat mean_a;
  boxFilter(a, temp1, -1, size_r);
  divide(temp1, N, mean_a);
  // mean_b = boxfilter(b, r) ./ N;
  Mat mean_b;
  boxFilter(b, temp1, -1, size_r);
  divide(temp1, N, mean_b);

  // q = mean_a .* I + mean_b; % Eqn. (8) in the paper;
  Mat q = mean_a.mul(guidance_img) + mean_b;
  return q;
}
