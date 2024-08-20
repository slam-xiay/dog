#include "blackboard.hpp"
BlackBoard::BlackBoard(){
    node_handle_ptr_ = std::make_shared<ros::NodeHandle>("");
    time_prys_ptr_ = std::make_shared<std::vector<TimePry>>();
}

std::shared_ptr<ros::NodeHandle> BlackBoard::GetNodeHandlePtr(){
    return node_handle_ptr_;
};

void BlackBoard::InsertTimePry(const TimePry& pry){
    boost::unique_lock<boost::shared_mutex> lock(time_prys_mutex_);
    TrimTimePrys();
    time_prys_ptr_->push_back(pry);
}

void BlackBoard::TrimTimePrys(){
    while(time_prys_ptr_->size()>=2 && ToSeconds(time_prys_ptr_->back().time-time_prys_ptr_->front().time)>1)
        time_prys_ptr_->erase(time_prys_ptr_->begin());
}
std::shared_ptr<std::vector<TimePry>> BlackBoard::GetTimePrysPtr(){    
    boost::shared_lock<boost::shared_mutex> lock(time_prys_mutex_);
    TrimTimePrys();
    return time_prys_ptr_;
}

