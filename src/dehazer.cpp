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
  int r = local_window_radius;
  // [hei, wid] = size(img);
  // filt_img = zeros(size(img));
  cv::Mat filt_img = cv::Mat::zeros(img.size(), CV_32FC1);

  // %cumulative sum over Y axis
  // imCum = cumsum(img, 1);
  cv::Mat cum;
  cv::reduce(img, cum, 0, CV_REDUCE_SUM);
  // %difference over Y axis
  // filt_img(1:r+1, :) = imCum(1+r:2*r+1, :);
  for (size_t i = 0; i < r; i++) {
    for (size_t j = 0; j < cum.cols; j++) {
      filt_img.at<float>(i, j) = cum.at<float>(r+i, j);
    }
  }

  // filt_img(r+2:hei-r, :) = imCum(2*r+2:hei, :) - imCum(1:hei-2*r-1, :);
  for (size_t i = 0; i < img.rows - 2*r - 1; i++) {
    for (size_t j = 0; j < cum.cols; j++) {
      filt_img.at<float>(i + r + 1, j) = cum.at<float>(i+2*r+1, j)
                                       - cum.at<float>(i, j);
    }
  }

  // filt_img(hei-r+1:hei, :) = repmat(imCum(hei, :), [r, 1])
  //                            - imCum(hei-2*r:hei-r-1, :);
  for (size_t i = 0; i < r; i++) {
    for (size_t j = 0; j < cum.cols; j++) {
      filt_img.at<float>(img.rows-r+i, j) = cum.at<float>(img.rows - 1, j)
        - cum.at<float>(img.rows - 2*r - 1 + i, j);
    }
  }


  // %cumulative sum over X axis
  // imCum = cumsum(filt_img, 2);
  cv::reduce(filt_img, cum, 1, CV_REDUCE_SUM);

  // %difference over Y axis
  // filt_img(:, 1:r+1) = imCum(:, 1+r:2*r+1);
  for (size_t i = 0; i < cum.rows; i++) {
    for (size_t j = 0; j < r; j++) {
      filt_img.at<float>(i, j) = cum.at<float>(i, r+j);
    }
  }

  // filt_img(:, r+2:wid-r) = imCum(:, 2*r+2:wid) - imCum(:, 1:wid-2*r-1);
  for (size_t i = 0; i < img.rows; i++) {
    for (size_t j = 0; j < img.cols - 2*r - 1; j++) {
      filt_img.at<float>(i, j + r + 1) = cum.at<float>(i, j+2*r+1)
                                       - cum.at<float>(i, j);
    }
  }

  // filt_img(:, wid-r+1:wid) = repmat(imCum(:, wid), [1, r])
  //                            - imCum(:, wid-2*r:wid-r-1);
  for (size_t i = 0; i < img.rows; i++) {
    for (size_t j = 0; j < r; j++) {
      filt_img.at<float>(i, img.cols - j) = cum.at<float>(i, img.cols - 1)
        - cum.at<float>(i, img.cols - 2*r - 1 + j);
    }
  }

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

// cv::Mat Dehazer::dehaze(cv::Mat img) {
//   ROS_INFO("Dehazing...");
//   cv::Mat dark_channel(img.size(), CV_32FC1);
//   for (size_t i = 0; i < img.rows; i++) {
//     for (size_t j = 0; j < img.cols; j++) {
//       cv::Vec3b colour = img.at<cv::Vec3b>(i, j);
//       int red = colour[2];
//       int green = colour[1];
//       int blue = colour[0];
//       int val = static_cast<float>(minimum(red, green, blue));
//       // ROS_INFO_STREAM("Max is " << val);
//       dark_channel.at<float>(i, j) = val;
//     }
//   }
//   // ROS_INFO_STREAM("black ( " << (int)black.rows << ", " << (int)black.cols << ")...");
//   double min_dark_channel, max_dark_channel;
//   cv::minMaxLoc(dark_channel, &min_dark_channel, &max_dark_channel);
//   ROS_INFO_STREAM("max_dark_channel is " << max_dark_channel);
//   cv::Mat ones = cv::Mat::ones(img.size(), CV_32FC1);
//   cv::divide(dark_channel, max_dark_channel, dark_channel);
//   // black = black / max_dark_channel;

//   // ROS_INFO_STREAM("ones ( " << (int)ones.rows << ", " << (int)ones.cols << ")...");
//   cv::Mat t = ones - 0.6*dark_channel; // if t < 0.1, t = 0.1

//   cv::namedWindow("Black", 0);
//   cv::imshow("Black", t);
//   cv::waitKey(3);

//   for (size_t i = 0; i < t.rows; i++) {
//     for (size_t j = 0; j < t.cols; j++) {
//       float val = t.at<float>(i, j);
//       if (val < 0.1)
//         t.at<float>(i, j) = 0.1;
//     }
//   }
//   /* First, we have to allocate the new channels */
//   cv::Mat r(img.rows, img.cols, CV_8UC1);
//   cv::Mat b(img.rows, img.cols, CV_8UC1);
//   cv::Mat g(img.rows, img.cols, CV_8UC1);

//   /* Now we put these into a matrix */
//   cv::Mat out[] = {b, g, r};

//   /* Split the image into the three color channels */
//   cv::split(img, out);
//   r.convertTo(r, CV_32FC1);
//   g.convertTo(g, CV_32FC1);
//   b.convertTo(b, CV_32FC1);
//   r = r - (ones-t)*max_dark_channel;
//   g = g - (ones-t)*max_dark_channel;
//   b = b - (ones-t)*max_dark_channel;
//   cv::divide(r, t, r);
//   cv::divide(g, t, g);
//   cv::divide(b, t, b);
//   r.convertTo(r, CV_8UC1);
//   g.convertTo(g, CV_8UC1);
//   b.convertTo(b, CV_8UC1);
//   cv::Mat dehazed(img.rows, img.cols, CV_8UC3);
//   cv::merge(out, 3, dehazed);
//   return dehazed;
// }
