#ifndef Multilinelaser_HPP
#define Multilinelaser_HPP
#include <glog/logging.h>
#include <signal.h>
#include <thread>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include "config.hpp"
#include <ros/ros.h>
#include <fcntl.h>
#include <poll.h>

class Multilinelaser
 {
  public:
  Multilinelaser(std::shared_ptr<ros::NodeHandle> node_handle_ptr);
  ~Multilinelaser();
  Multilinelaser(const Multilinelaser &) = delete;
  Multilinelaser &operator=(const Multilinelaser &) = delete;
  void MultilinelaserRxThread();
  private:
  int fd_{-1};
  sockaddr_in address_;

  std::shared_ptr<ros::NodeHandle> node_handle_ptr_;
  ros::Publisher publisher_;
  ros::ServiceServer switch_;
  std::string name_;
  bool is_running_{false};


};

#endif 
