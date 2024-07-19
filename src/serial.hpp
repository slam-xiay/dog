#ifndef SERIAL_HPP_
#define SERIAL_HPP_

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>

#include <string>
#include <vector>

#include "config.hpp"

class Serial {
 public:
  Serial(const std::string& serial_port, const int& baudrate_code_);
  ~Serial();
  bool Open();
  bool Close();
  bool ReadFromIO(std::vector<uint8_t>& data);
  bool WriteToIO(const std::vector<uint8_t>& data);
  bool WriteToIOWithoutCrc(const std::vector<uint8_t>& data);
  // bool ReadFromIO(uint8_t* rx_buf, size_t& rx_len);
  bool WriteToIO(const uint8_t* tx_buf, const size_t& tx_len);
  // bool ReadFromIO(uint8_t* rx_buf, int32_t& rx_len);
  bool WriteToIO(const uint8_t* tx_buf, const uint32_t& tx_len);
  bool is_open_{false};

 private:
  int fd_{-1};
  std::string serial_port_;
  int baudrate_code_;
};

#endif