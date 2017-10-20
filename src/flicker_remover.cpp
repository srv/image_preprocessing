#include "image_preprocessing/flicker_remover.h"

FlickerRemover::FlickerRemover(ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nhp_(nhp), frame_counter_(0) {}

void FlickerRemover::run()
{
  // Read parameters
  string camera_topic;
  nhp_.param("camera_topic", camera_topic, string(""));
  nhp_.param("desc_thresh", desc_thresh_, string(""));
  nhp_.param("epipolar_thresh", epipolar_thresh_, string(""));
  nhp_.param("min_inliers", min_inliers_, string(""));


  // Message sync
  boost::shared_ptr<Sync> sync;
  image_transport::ImageTransport it(nh_);
  image_transport::SubscriberFilter left_sub, right_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub, right_info_sub;
  left_sub      .subscribe(it, camera_topic+"/left/image_rect_color", 5);
  right_sub     .subscribe(it, camera_topic+"/right/image_rect_color", 5);
  left_info_sub .subscribe(nh_, camera_topic+"/left/camera_info",  5);
  right_info_sub.subscribe(nh_, camera_topic+"/right/camera_info", 5);
  sync.reset(new Sync(SyncPolicy(10), left_sub, right_sub, left_info_sub, right_info_sub) );
  sync->registerCallback(bind(&FlickerRemover::msgsCallback, this, _1, _2, _3, _4));

  ros::spin();
}

void FlickerRemover::msgsCallback(
      const sensor_msgs::ImageConstPtr& l_img_msg,
      const sensor_msgs::ImageConstPtr& r_img_msg,
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
  // Convert images to opencv
  cv::Mat l_img, r_img;
  bool convert_valid = imgMsgToMat(*l_img_msg, *r_img_msg, l_img, r_img);
  if (!convert_valid)
  {
    ROS_WARN("[FlickerRemover:] Impossible to convert images to opencv");
    return;
  }

  // Proceed depending if system is initilized or not
  if (frame_counter_ == 0)
  {
    prev_l_ = l_img;
    prev_r_ = r_img;
    frame_counter_++;
    return;
  }
  else if (frame_counter_ == 1)
  {
    curr_l_ = l_img;
    curr_r_ = r_img;
    frame_counter_++;
    return;
  }
  else
  {
    next_l_ = l_img;
    next_r_ = r_img;
    frame_counter_++;

    // Register the set of images
    registerImages();


    // Update
    prev_l_ = curr_l_;
    prev_r_ = curr_r_;
    curr_l_ = next_l_;
    curr_r_ = next_r_;
  }
}

void FlickerRemover::registerImages()
{
  vector<cv::KeyPoint> next_l_kp, next_r_kp;
  vector<cv::KeyPoint> curr_l_kp, curr_r_kp;
  vector<cv::KeyPoint> prev_l_kp, prev_r_kp;

  cv::Mat next_l_desc, next_r_desc;
  cv::Mat curr_l_desc, curr_r_desc;
  cv::Mat prev_l_desc, prev_r_desc;

  // Extract features
  featureExtraction(next_l_, next_l_kp, next_l_desc);
  featureExtraction(next_r_, next_r_kp, next_r_desc);
  featureExtraction(curr_l_, curr_l_kp, curr_l_desc);
  featureExtraction(curr_r_, curr_r_kp, curr_r_desc);
  featureExtraction(prev_l_, prev_l_kp, prev_l_desc);
  featureExtraction(prev_r_, prev_r_kp, prev_r_desc);

  // Feature matching
  vector<cv::DMatch> nl_nr_matches, nl_cl_matches, nl_cr_matches;
  vector<cv::DMatch> nl_pl_matches, nl_pr_matches, nr_cl_matches;
  vector<cv::DMatch> nr_cr_matches, nr_pl_matches, nr_pr_matches;
  vector<cv::DMatch> cl_cr_matches, cl_pl_matches, cl_pr_matches;
  vector<cv::DMatch> cr_pl_matches, cr_pr_matches, pl_pr_matches;
  crossCheckThresholdMatching(next_l_desc, next_r_desc, 0.8, nl_nr_matches);
  crossCheckThresholdMatching(next_l_desc, curr_l_desc, 0.8, nl_cl_matches);
  crossCheckThresholdMatching(next_l_desc, curr_r_desc, 0.8, nl_cr_matches);
  crossCheckThresholdMatching(next_l_desc, prev_l_desc, 0.8, nl_pl_matches);
  crossCheckThresholdMatching(next_l_desc, prev_r_desc, 0.8, nl_pr_matches);
  crossCheckThresholdMatching(next_r_desc, curr_l_desc, 0.8, nr_cl_matches);
  crossCheckThresholdMatching(next_r_desc, curr_r_desc, 0.8, nr_cr_matches);
  crossCheckThresholdMatching(next_r_desc, prev_l_desc, 0.8, nr_pl_matches);
  crossCheckThresholdMatching(next_r_desc, prev_r_desc, 0.8, nr_pr_matches);
  crossCheckThresholdMatching(curr_l_desc, curr_r_desc, 0.8, cl_cr_matches);
  crossCheckThresholdMatching(curr_l_desc, prev_l_desc, 0.8, cl_pl_matches);
  crossCheckThresholdMatching(curr_l_desc, prev_r_desc, 0.8, cl_pr_matches);
  crossCheckThresholdMatching(curr_r_desc, prev_l_desc, 0.8, cr_pl_matches);
  crossCheckThresholdMatching(curr_r_desc, prev_r_desc, 0.8, cr_pr_matches);
  crossCheckThresholdMatching(prev_l_desc, prev_r_desc, 0.8, pl_pr_matches);

  // Registration
  cv::Mat nl_nr, nl_cl, nl_cr, nl_pl, nl_pr;
  cv::Mat nr_cl, nr_cr, nr_pl, nr_pr;
  cv::Mat cl_cr, cl_pl, cl_pr;
  cv::Mat cr_pl, cr_pr;
  cv::Mat pl_pr;


  ROS_INFO("-------------------------------");
  ROS_INFO("-------------------------------");
  ROS_INFO_STREAM("nl_nr: " << nl_nr_matches.size());
  ROS_INFO_STREAM("nl_cl: " << nl_cl_matches.size());
  ROS_INFO_STREAM("nl_cr: " << nl_cr_matches.size());
  ROS_INFO_STREAM("nl_pl: " << nl_pl_matches.size());
  ROS_INFO_STREAM("nl_pr: " << nl_pr_matches.size());
  ROS_INFO_STREAM("nr_cl: " << nr_cl_matches.size());
  ROS_INFO_STREAM("nr_cr: " << nr_cr_matches.size());
  ROS_INFO_STREAM("nr_pl: " << nr_pl_matches.size());
  ROS_INFO_STREAM("nr_pr: " << nr_pr_matches.size());
  ROS_INFO_STREAM("cl_cr: " << cl_cr_matches.size());
  ROS_INFO_STREAM("cl_pl: " << cl_pl_matches.size());
  ROS_INFO_STREAM("cl_pr: " << cl_pr_matches.size());
  ROS_INFO_STREAM("cr_pl: " << cr_pl_matches.size());
  ROS_INFO_STREAM("cr_pr: " << cr_pr_matches.size());
  ROS_INFO_STREAM("pl_pr: " << pl_pr_matches.size());

}

void FlickerRemover::register(vector<cv::DMatch> matches, )
{
  if (matches.size() >= min_inliers_)
  {
    vector<Point2f> matched_a;
    vector<Point2f> matched_b;
    for(int i=0; i<matches.size(); i++)
    {
      matched_a.push_back(query_kp[desc_matches[i].trainIdx].pt);
      matched_b.push_back(candidate_kp[desc_matches[i].queryIdx].pt);
    }
    nl_nr = findFundamentalMat(matched_b, matched_a, FM_RANSAC, params_.epipolar_thresh, 0.999, status);
  }
}

void FlickerRemover::featureExtraction(cv::Mat img, vector<cv::KeyPoint>& kp, cv::Mat& desc)
{
  kp.clear();
  desc.release();
  cv::Ptr<cv::Feature2D> orb;
  orb = cv::ORB::create(2000, 1.2, 8, 10, 0, 2, cv::ORB::HARRIS_SCORE, 10);
  orb->detectAndCompute (img, cv::noArray(), kp, desc);
}

bool FlickerRemover::imgMsgToMat(sensor_msgs::Image l_img_msg,
                                 sensor_msgs::Image r_img_msg,
                                 cv::Mat &l_img, cv::Mat &r_img)
{
  // Convert message to cv::Mat
  cv_bridge::CvImagePtr l_img_ptr, r_img_ptr;
  try
  {
    l_img_ptr = cv_bridge::toCvCopy(l_img_msg, enc::BGR8);
    r_img_ptr = cv_bridge::toCvCopy(r_img_msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("[FlickerRemover:] cv_bridge exception: %s", e.what());
    return false;
  }

  // Set the images
  l_img = l_img_ptr->image;
  r_img = r_img_ptr->image;
  return true;
}

void FlickerRemover::thresholdMatching(const cv::Mat& descriptors1, const cv::Mat& descriptors2,
  double threshold, vector<cv::DMatch>& matches)
{
  cv::Mat match_mask;
  matches.clear();
  if (descriptors1.empty() || descriptors2.empty())
    return;
  assert(descriptors1.type() == descriptors2.type());
  assert(descriptors1.cols == descriptors2.cols);

  const int knn = 2;
  cv::Ptr<cv::DescriptorMatcher> descriptor_matcher;
  // choose matcher based on feature type
  if (descriptors1.type() == CV_8U)
  {
    descriptor_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
  }
  else
  {
    descriptor_matcher = cv::DescriptorMatcher::create("BruteForce");
  }
  vector<vector<cv::DMatch> > knn_matches;
  descriptor_matcher->knnMatch(descriptors1, descriptors2,
          knn_matches, knn, match_mask);

  for (size_t m = 0; m < knn_matches.size(); m++)
  {
    if (knn_matches[m].size() < 2) continue;
    float dist1 = knn_matches[m][0].distance;
    float dist2 = knn_matches[m][1].distance;
    if (dist1 / dist2 < threshold)
    {
      matches.push_back(knn_matches[m][0]);
    }
  }
}

void FlickerRemover::crossCheckFilter(
    const vector<cv::DMatch>& matches1to2,
    const vector<cv::DMatch>& matches2to1,
    vector<cv::DMatch>& checked_matches)
{
  checked_matches.clear();
  for (size_t i = 0; i < matches1to2.size(); ++i)
  {
    bool match_found = false;
    const cv::DMatch& forward_match = matches1to2[i];
    for (size_t j = 0; j < matches2to1.size() && match_found == false; ++j)
    {
      const cv::DMatch& backward_match = matches2to1[j];
      if (forward_match.trainIdx == backward_match.queryIdx &&
          forward_match.queryIdx == backward_match.trainIdx)
      {
        checked_matches.push_back(forward_match);
        match_found = true;
      }
    }
  }
}

void FlickerRemover::crossCheckThresholdMatching(
  const cv::Mat& descriptors1, const cv::Mat& descriptors2,
  double threshold, const cv::Mat& match_mask,
  vector<cv::DMatch>& matches)
{
  vector<cv::DMatch> query_to_train_matches;
  thresholdMatching(descriptors1, descriptors2, threshold, match_mask, query_to_train_matches);
  vector<cv::DMatch> train_to_query_matches;
  cv::Mat match_mask_t;
  if (!match_mask.empty()) match_mask_t = match_mask.t();
  thresholdMatching(descriptors2, descriptors1, threshold, match_mask_t, train_to_query_matches);

  crossCheckFilter(query_to_train_matches, train_to_query_matches, matches);
}