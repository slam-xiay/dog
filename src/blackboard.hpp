#ifndef BLACKBOARD_HPP_
#define BLACKBOARD_HPP_
// #include <sensor_msgs/PointCloud.h>
#include <map>
#include <boost/thread/pthread/shared_mutex.hpp>
#include "common/time.hpp"
#include "common/time_pry.hpp"
class BlackBoard {
    public:
        BlackBoard();
        ~BlackBoard()=default;
        BlackBoard(const BlackBoard&) = delete;
        BlackBoard& operator=(const BlackBoard&) = delete;
        
        void InsertTimePry(const TimePry& pry);
        std::shared_ptr<std::vector<TimePry>> GetTimePrysPtr();
        void TrimTimePrys();

        std::shared_ptr<ros::NodeHandle> GetNodeHandlePtr();

    private:
        std::shared_ptr<ros::NodeHandle> node_handle_ptr_;
        std::shared_ptr<std::vector<TimePry>> time_prys_ptr_;     
        boost::shared_mutex time_prys_mutex_; 

};
#endif