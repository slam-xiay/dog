#ifndef BLACKBOARD_HPP_
#define BLACKBOARD_HPP_
// #include <sensor_msgs/PointCloud.h>
#include <map>
#include <boost/thread/pthread/shared_mutex.hpp>
#include "common/time.hpp"
#include "common/time_rpy.hpp"
#include "common/time_cloud.hpp"
class BlackBoard {
    public:
        BlackBoard();
        ~BlackBoard()=default;
        BlackBoard(const BlackBoard&) = delete;
        BlackBoard& operator=(const BlackBoard&) = delete;

        std::shared_ptr<ros::NodeHandle> GetNodeHandlePtr();
        
        void InsertTimeRpy(const TimeRpy& rpy);

        std::shared_ptr<std::vector<TimeRpy>> GetTimeRpysPtr();

        void TrimTimeRpys();

        bool GetLatestRpy(Eigen::Vector3f& rpy);

        void InsertTimeAddressCloud(const TimeAddressCloud& time_address_cloud);

        void TrimTimeAddressCloud();

        std::shared_ptr<std::vector<TimeAddressCloud>> GetTimeAddressCloudPtr();

        bool GetLatestAddressCloud(std::map<uint32_t,Eigen::Vector3f>&);

    private:
        std::shared_ptr<ros::NodeHandle> node_handle_ptr_;

        boost::shared_mutex time_rpys_mutex_; 
        std::shared_ptr<std::vector<TimeRpy>> time_rpys_ptr_;                     

        boost::shared_mutex address_clouds_mutex_;
        std::shared_ptr<std::vector<TimeAddressCloud>> address_clouds_ptr_;   

};
#endif