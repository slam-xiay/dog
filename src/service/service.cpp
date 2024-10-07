#include "service.hpp"
Service::Service(std::shared_ptr<BlackBoard> black_board_ptr):black_board_ptr_(black_board_ptr) {
  docker_detector_ptr_ = std::make_shared<DockerDetector>(black_board_ptr);
};
Service::~Service() {
  
};