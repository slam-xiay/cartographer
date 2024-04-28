#include <glog/logging.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::SetLogDestination(google::GLOG_INFO, "/root/gamma/log/log.txt");
  ros::init(argc, argv, "slam");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    LOG(ERROR) << "Sleep.";
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}