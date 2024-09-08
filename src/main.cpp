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

#include "robot.hpp"

int main(int argc, char *argv[]) {
  // uint8_t a = uint8_t(int8_t(201));
  // uint8_t b = uint8_t(int8_t(200));
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  ros::init(argc, argv, "robot");
  std::unique_ptr<Robot> robot_ptr = std::make_unique<Robot>();
  ros::spin();
  return 0;
}
