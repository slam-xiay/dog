#include "serial.hpp"
Serial::Serial(const std::string& serial_port)
    : serial_port_(serial_port){};
Serial::~Serial() { Close(); };
bool Serial::Open(const int& baudrate_code) {
  fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ == -1) {
    LOG(ERROR) << "Failure to open(" << serial_port_ << ");";
    return false;
  } else {
    LOG(ERROR) << "Succeeful to open(" << serial_port_ << "),fd:(" << fd_
               << ");";
    fcntl(fd_, F_SETFL, FNDELAY);
    struct termios options;
    options.c_cflag = baudrate_code | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    int ret = tcsetattr(fd_, TCSANOW, &options);
    if (ret != 0) return false;
    tcflush(fd_, TCIFLUSH);
    return true;
  }
};
bool Serial::Close() {
  if (fd_ >= 0) {
    if (close(fd_) != 0) {
      LOG(ERROR) << "Close serial(" << serial_port_ << ") fail.";
      return false;
    } else {
      fd_ = -1;
      LOG(ERROR) << "Close serial(" << serial_port_ << ") successful.";
      return true;
    }
  } else {
    LOG(ERROR) << "Serial is closed aleardy.";
    return true;
  }
};

bool Serial::ReadFromIO(std::vector<uint8_t>& data) {
  LOG_EVERY_N(ERROR,30)<<"ReadFromIO fd_:("<<fd_<<").";
  if (fd_ == -1) return false;
  uint8_t* rx_buf = new uint8_t[kMaxSerialBuf];
  size_t rx_len = 0;
  static timespec timeout = {0, (long)(kPerReadTimeoutns)};  // 0.1ms
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(fd_, &read_fds);
  int r = pselect(fd_ + 1, &read_fds, NULL, NULL, &timeout, NULL);
  if (r <= 0) return false;
  if (FD_ISSET(fd_, &read_fds)) rx_len = read(fd_, rx_buf, kMaxSerialBuf);
  for (int i = 0; i < rx_len; i++) data.push_back(*(rx_buf + i));
  // LOG(ERROR) << "rx_len:(" << rx_len << ").";
  // printf("\n");
  // printf("rx_len(%d)", rx_len);
  // printf("========\n");
  // for (auto&& byte : data) {
  //   printf("%02x ", byte);
  // }
  // printf("========\n");
  // LOG(ERROR) <<fd_<< " Read length:(" << rx_len << "),data size(" << data.size()
  //            << ").";
  return rx_len > 0;
}

bool Serial::WriteToIO(const std::vector<uint8_t>& data) {
  if (fd_ == -1) return false;
  uint8_t tx_buf[data.size()];
  std::copy(data.begin(),data.end(),tx_buf);
  size_t tx_len = size_t(data.size());
  int ret = write(fd_, &tx_buf, tx_len);
  return (ret > 0);
}
// bool Serial::WriteToIOWithoutCrc(const std::vector<uint8_t>& data) {
//   if (fd_ < 0) return false;
//   auto filtered_data = data;
//   uint8_t crc = 0;
//   for (auto&& byte : data) crc += byte;
//   filtered_data.push_back(crc);
//   return WriteToIO(filtered_data);
// }