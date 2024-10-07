#include "blackboard.hpp"
BlackBoard::BlackBoard(){
    node_handle_ptr_ = std::make_shared<ros::NodeHandle>("");
    time_rpys_ptr_ = std::make_shared<std::vector<TimeRpy>>();
    address_clouds_ptr_ = std::make_shared<std::vector<TimeAddressCloud>>();
}

std::shared_ptr<ros::NodeHandle> BlackBoard::GetNodeHandlePtr(){
    return node_handle_ptr_;
};

void BlackBoard::InsertTimeRpy(const TimeRpy& rpy){
    boost::unique_lock<boost::shared_mutex> lock(time_rpys_mutex_);
    time_rpys_ptr_->push_back(rpy);
    TrimTimeRpys();
}

void BlackBoard::TrimTimeRpys(){
    while(time_rpys_ptr_->size()>=2 && ToSeconds(time_rpys_ptr_->back().time-time_rpys_ptr_->front().time)>1)
        time_rpys_ptr_->erase(time_rpys_ptr_->begin());
}
std::shared_ptr<std::vector<TimeRpy>> BlackBoard::GetTimeRpysPtr(){    
    boost::shared_lock<boost::shared_mutex> lock(time_rpys_mutex_);
    TrimTimeRpys();
    return time_rpys_ptr_;
}

bool BlackBoard::GetLatestRpy(Eigen::Vector3f& rpy){
    std::shared_ptr<std::vector<TimeRpy>>  time_rpys_ptr = GetTimeRpysPtr();
    if(time_rpys_ptr->empty()){
        return false;
    }
    else{
        rpy = time_rpys_ptr->back().rpy;
        return true;
    }
}

void BlackBoard::InsertTimeAddressCloud(const TimeAddressCloud& time_address_cloud){
    boost::unique_lock<boost::shared_mutex> lock(address_clouds_mutex_);
    address_clouds_ptr_->push_back(time_address_cloud);
}

void BlackBoard::TrimTimeAddressCloud(){
    boost::unique_lock<boost::shared_mutex> lock(address_clouds_mutex_);
    while(address_clouds_ptr_->size()>=2 && ToSeconds(address_clouds_ptr_->back().time-address_clouds_ptr_->front().time)>1)
        address_clouds_ptr_->erase(address_clouds_ptr_->begin());
}

std::shared_ptr<std::vector<TimeAddressCloud>> BlackBoard::GetTimeAddressCloudPtr(){    
    boost::shared_lock<boost::shared_mutex> lock(address_clouds_mutex_);
    TrimTimeAddressCloud();
    return address_clouds_ptr_;
}

bool BlackBoard::GetLatestAddressCloud(std::map<uint32_t,Eigen::Vector3f>& address_cloud){
    std::shared_ptr<std::vector<TimeAddressCloud>>  time_address_cloud_ptr = GetTimeAddressCloudPtr();
    if(time_address_cloud_ptr->empty()){
        return false;
    }
    else{
        address_cloud = time_address_cloud_ptr->back().address_cloud;
        return true;
    }
}
