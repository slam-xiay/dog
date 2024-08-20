#include "robot.hpp"

Robot::Robot() {
    black_board_ptr_ = std::make_shared<BlackBoard>();
    drivers_ptr_ = std::make_shared<Drivers>(black_board_ptr_);
    OpenRobot();
};

Robot::~Robot() { CloseRobot(); };

bool Robot::OpenRobot() {
    return true;
};

bool Robot::CloseRobot() {
    return true;
};


   