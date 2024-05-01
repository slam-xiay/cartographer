#include <glog/logging.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "ros/slam.h"
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "slam");
  google::InitGoogleLogging(argv[0]);
  google::SetLogDestination(google::GLOG_INFO, "/root/gamma/log/log.txt");
  std::unique_ptr<cartographer::slam::Slam> slam_ptr =
      std::make_unique<cartographer::slam::Slam>();
  ros::spin();
  return 0;
}