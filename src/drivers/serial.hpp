#ifndef DRIVERS_SERIAL_HPP_
#define DRIVERS_SERIAL_HPP_

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <string>
#include <vector>

#include "config.hpp"

class Serial {
 public:
  Serial(const std::string& serial_port);
  ~Serial();
  bool Open(const int& baudrate_code);
  bool Close();
  bool ReadFromIO(std::vector<uint8_t>& data);
  bool WriteToIO(const std::vector<uint8_t>& data);
  int fd_{-1};

 private:
  std::string serial_port_;
};

#endif