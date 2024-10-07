#ifndef LINELASER_HPP
#define LINELASER_HPP
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/SetBool.h>

#include "serial.hpp"

struct LinelaserDataFrame {
  uint8_t header[4];  // 4
  uint8_t address;    // 5
  uint8_t command;    // 6
  uint8_t length[2];  // 8
  uint8_t data[322];  // 330
  uint8_t crc;        // 331
};

struct LinelaserCommandFrame {
  uint8_t header[4];
  uint8_t address;
  uint8_t command;
  uint8_t length[2];
  uint8_t crc;
};

class Linelaser : public Serial {
 public:
  Linelaser(std::shared_ptr<ros::NodeHandle> node_handle_ptr,
            const std::string &serial_port);
  ~Linelaser() = default;
  Linelaser(const Linelaser &) = default;
  Linelaser &operator=(const Linelaser &) = default;

  void LinelaserRxThread();

  void ParseData(std::vector<uint8_t> &data);

  void Publish(const std::string& name,const float &range) ;

  bool SetRunning(const bool &running);

  bool SwitchCallBack(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res);

 private:
  std::shared_ptr<ros::NodeHandle> node_handle_ptr_;
  ros::Publisher publisher_;
  ros::ServiceServer switch_;
  std::string name_;
  bool is_running_{false};  // 是否运行
  bool is_start_{false};    // 是否有数据
  bool is_right_bandrate_{false};
};
#endif