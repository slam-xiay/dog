#ifndef SLAM_COMMON_time_pry_HPP__
#define SLAM_COMMON_time_pry_HPP__
#include <Eigen/Dense>

#include "config.hpp"
#include "time.hpp"

struct TimePry {
    Time            time;
    Eigen::Vector3f pry;
    TimePry() : time(GetNow()), pry(Eigen::Vector3f::Zero()){};
    TimePry(Time time_, Eigen::Vector3f pry_) {
        time = time_;
        pry = pry_;
    };
};

#endif