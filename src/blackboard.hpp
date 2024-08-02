#ifndef BLACKBOARD_HPP_
#define BLACKBOARD_HPP_
#include <sensor_msgs/PointCloud.h>
#include <map>
#include <mutex>
class BlackBoard {
    public:
        BlackBoard();
        ~BlackBoard();
        BlackBoard(const BlackBoard&) = delete;
        BlackBoard& operator=(const BlackBoard&) = delete;
    private:
        
}   
#endif