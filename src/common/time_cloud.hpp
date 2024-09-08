#ifndef SLAM_COMMON_TIME_ADDRESS_CLOUD_HPP__
#define SLAM_COMMON_TIME_ADDRESS_CLOUD_HPP__
#include <Eigen/Dense>
#include <map>
#include "config.hpp"
#include "time.hpp"

struct TimeAddressCloud {
    Time            time;
    std::map<uint32_t,Eigen::Vector3f> address_cloud;    
    TimeAddressCloud() : time(GetNow()){};
    TimeAddressCloud(Time time_instance, 
        std::map<uint32_t,Eigen::Vector3f> address_cloud_instance) {
        time = time_instance;
        address_cloud = address_cloud_instance;
    };
};

#endif