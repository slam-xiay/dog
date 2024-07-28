#ifndef LINELASER_HPP
#define LINELASER_HPP
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/SetBool.h>

#include "serial.hpp"

struct LinelaserFrame {
  uint32_t header;
  uint8_t address;
  uint8_t command;
  uint16_t length;
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

  bool ParseData(std::vector<uint8_t> &data);

  void Publish(const float &range);

  void SetRunning(const bool &running);

  bool SwitchCallBack(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res);

  bool SetBandrate(const int &bandrate_code);

  bool SetStartCommand();

  const std::vector<uint8_t> kLinelaserGetAddressCommand = {
      0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x60, 0x00, 0x00, 0x60};

  const std::vector<uint8_t> kLinelaserGetParameterCommand = {
      0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x61, 0x00, 0x00, 0x61};

  const std::vector<uint8_t> kLinelaserGetVersionCommand = {
      0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x62, 0x00, 0x00, 0x62};

  const std::vector<uint8_t> kLinelaserStartCommand = {
      0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x63, 0x00, 0x00, 0x63};

  const std::vector<uint8_t> kLinelaserStopCommand = {
      0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x64, 0x00, 0x00, 0x64};

  const std::vector<uint8_t> kLinelaserRestartCommand = {
      0xA5, 0xA5, 0xA5, 0xA5, 0x01, 0x67, 0x00, 0x00, 0x68};

  const std::vector<uint8_t> kLinelaserSet921600 = {
      0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x68, 0x01, 0x00, 0x03, 0x6C};

  const std::vector<uint8_t> kLinelaserSet230400 = {
      0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x68, 0x01, 0x00, 0x01, 0x6A};

 private:
  std::shared_ptr<ros::NodeHandle> node_handle_ptr_;
  ros::Publisher publisher_;
  ros::ServiceServer switch_;
  std::string name_;
  bool is_running_{true};
};
#endif