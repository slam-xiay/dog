#include <errno.h>
#include <glog/logging.h>
#include <malloc.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include <iostream>
#include <thread>

#include "drivers.hpp"

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  ros::init(argc, argv, "drivers");
  std::unique_ptr<Drivers> robot_ptr = std::make_unique<Drivers>();
  ros::spin();
  return 0;
}
