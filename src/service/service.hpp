#ifndef SERVICE_SERVICE_HPP
#define SERVICE_SERVICE_HPP
#include <glog/logging.h>
#include <ros/ros.h>
#include <thread>
#include <vector>
#include "docker_detector.hpp"
#include "../blackboard.hpp"

class Service {
 public:
  Service(std::shared_ptr<BlackBoard> black_board_ptr_);
  ~Service();
  Service(const Service &) = delete;
  Service &operator=(const Service &) = delete;

 private:
  std::shared_ptr<BlackBoard> black_board_ptr_;  
  std::shared_ptr<DockerDetector> docker_detector_ptr_;
};

#endif // !Service_HPP
