#ifndef ROBOT_HPP_
#define ROBOT_HPP_
#include <execinfo.h>
#include <signal.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "blackboard.hpp"
#include "drivers/drivers.hpp"


class Robot {
public:
    Robot();
    ~Robot();
    Robot(const Robot&) = delete;
    Robot&                             operator=(const Robot&) = delete;
    bool                               OpenRobot();
    bool                               CloseRobot();
    std::shared_ptr<BlackBoard>& GetBlackBoardPtr() { return black_board_ptr_; };

private:
    std::shared_ptr<BlackBoard> black_board_ptr_;
    std::shared_ptr<Drivers> drivers_ptr_;
};

#endif