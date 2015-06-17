#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <functional>
#include <boost/function.hpp>

#include <signal.h>

using namespace std;
using namespace cv;

Mat vig;
bool first = true;
double image_counter = 0.0;
// Stop handler binding
boost::function<void(int)> stopHandlerCb;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    Mat imgd;
    img.convertTo(imgd, CV_64FC3);
    if (first)
    {
      ROS_INFO("Images received correctly.");
      first = false;
      vig = Mat::zeros(img.size(), CV_64FC3);
    }
    vig = vig + imgd;
    image_counter++;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

/** \brief Catches the Ctrl+C signal.
  */
void stopHandler(int s)
{
  printf("Caught signal %d\n",s);
  stopHandlerCb(s);
  ros::shutdown();
}

void finalize(int s)
{
  ROS_INFO("Finalizing...");
  if (image_counter > 0)
  {
    vig = vig / image_counter;
    imwrite("vignetting.png", vig);
    ROS_INFO("Image saved!");
  }
  ROS_INFO("Done!");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vignetting_extractor");
  ros::NodeHandle nh;

  // Bind the finalize member to stopHandler signal
  stopHandlerCb = &finalize;
  // Setup the signal handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = stopHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);
  ros::spin();
}
