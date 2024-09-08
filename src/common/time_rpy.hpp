#ifndef SLAM_COMMON_TIME_RPY_HPP__
#define SLAM_COMMON_TIME_RPY_HPP__
#include <Eigen/Dense>

#include "config.hpp"
#include "time.hpp"

struct TimeRpy {
    Time            time;
    Eigen::Vector3f rpy;
    TimeRpy() : time(GetNow()), rpy(Eigen::Vector3f::Zero()){};
    TimeRpy(Time time_instance, Eigen::Vector3f rpy_instance) {
        time = time_instance;
        rpy = rpy_instance;
    };
};

#endif