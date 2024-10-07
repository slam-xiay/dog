#ifndef DRIVERS_DRIVERS_HPP
#define DRIVERS_DRIVERS_HPP
#include <glog/logging.h>
#include <ros/ros.h>

#include <thread>
#include <vector>

#include "linelaser.hpp"
#include "ultrasonic.hpp"
#include "multilinelaser.hpp"
#include "imu.hpp"
#include "../blackboard.hpp"

class Drivers {
 public:
  Drivers(std::shared_ptr<BlackBoard> black_board_ptr_);
  ~Drivers();
  Drivers(const Drivers &) = delete;
  Drivers &operator=(const Drivers &) = delete;

 private:
  std::shared_ptr<BlackBoard> black_board_ptr_;  
  // std::vector<std::shared_ptr<Ultrasonic>> ultrasonic_ptrs_;
  // std::vector<std::shared_ptr<std::thread>> ultrasonic_thread_ptrs_;
  // std::vector<std::shared_ptr<Linelaser>> linelaser_ptrs_;
  // std::vector<std::shared_ptr<std::thread>> linelaser_thread_ptrs_;
  std::shared_ptr<Multilinelaser> multilinelaser_ptr_;
  std::shared_ptr<std::thread> multilinelaser_thread_ptr_;
  std::shared_ptr<Imu> imu_ptr_;
  std::shared_ptr<std::thread> imu_thread_ptr_;
};

#endif // !DRIVERS_HPP
