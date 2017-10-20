#include <ros/ros.h>
#include <boost/thread.hpp>
#include <image_preprocessing/flicker_remover.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "clahs");
  ros::start();

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  FlickerRemover fr(nh, nhp);
  boost::thread trackingThread(&FlickerRemover::run, &fr);

  // ROS spin
  ros::Rate r(10);
  while (ros::ok())
  {
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
