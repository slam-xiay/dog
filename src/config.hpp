#ifndef CONFIG_HPP_
#define CONFIG_HPP_
#include <glog/logging.h>
#include <termios.h>
constexpr float kPI = 3.1415926f;
template <typename T>
T DegToRad(const T& deg) {
  return T(kPI * deg / 180.);
};
template <typename T>
T RadToDeg(const T& rad) {
  return T(180. * rad / kPI);
};

template <typename T>
T RadRange(const T& rad) {
  T result = rad;
  while (result >= T(2 * kPI)) result -= 2 * kPI;
  while (result < T(0)) result += 2 * kPI;
  return result;
}

template <typename T>
T RadNorm(const T& rad) {
  T result = rad;
  while (result >= T(kPI)) result -= 2 * kPI;
  while (result < T(-kPI)) result += 2 * kPI;
  return result;
}

constexpr termios kUltrasonicSerialOptions = {
    .c_iflag = uint16_t(IGNPAR) |
               uint16_t(~IGNBRK) & uint16_t(~BRKINT) & uint16_t(~PARMRK) &
                   uint16_t(~INPCK) & uint16_t(~ISTRIP) & uint16_t(~INLCR) &
                   uint16_t(~IGNCR) & uint16_t(~ICRNL) & uint16_t(~IUCLC) &
                   uint16_t(~IXON) & uint16_t(~IXANY) & uint16_t(~IXOFF) &
                   uint16_t(~IMAXBEL) & uint16_t(~IUTF8),
    .c_oflag = uint16_t(ONLCR) | uint16_t(NLDLY & NL0) | uint16_t(CRDLY & CR0) |
               uint16_t(TABDLY & TAB0) | uint16_t(BSDLY & BS0) |
               uint16_t(VTDLY & VT0) |
               uint16_t(FFDLY & FF0) & uint16_t(~OPOST) & uint16_t(~OLCUC) &
                   uint16_t(~OCRNL) & uint16_t(~ONOCR) & uint16_t(~ONLRET) &
                   uint16_t(~OFILL) & (uint16_t)~OFDEL,
    .c_cflag = uint16_t(B115200) | uint16_t(CS8) | uint16_t(CREAD) |
               uint16_t(HUPCL) |
               uint16_t(CLOCAL) & uint16_t(~CSTOPB) & uint16_t(~PARENB) &
                   uint16_t(~PARODD) |
               CRTSCTS,
    .c_lflag = uint16_t(ECHOCTL) | uint16_t(PENDIN) |
               uint16_t(ECHOKE) & uint16_t(~ECHO) & uint16_t(~ICANON) &
                   uint16_t(~ISIG) & uint16_t(~XCASE) & uint16_t(~ECHOE) &
                   uint16_t(~ECHOK) & uint16_t(~ECHONL) & uint16_t(~ECHOPRT) &
                   uint16_t(~FLUSHO) & uint16_t(~NOFLSH) & uint16_t(~TOSTOP) &
                   uint16_t(~IEXTEN),
    .c_line = uint8_t(0),
    .c_cc = {1, 1, 8, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0},
};

constexpr int64_t kPerReadTimeoutns = 100000;  // ns 0.1ms
constexpr int kMaxSerialBuf = 115200;
constexpr uint8_t kUltrasonicFrameHeader = 0xFF;
constexpr uint8_t kUltrasonicStartCode = 0xAA;
const std::vector<std::string> kUltrasonicPorts = {
    "/dev/Ultrasonic_fl", "/dev/Ultrasonic_fr", "/dev/Ultrasonic_bl",
    "/dev/Ultrasonic_br", "/dev/Ultrasonic_l",  "/dev/Ultrasonic_r"};
constexpr int kUltrasonicFrequence = 30;

const std::vector<std::string> kLinelaserPorts = {
    "/dev/line_laser_l", "/dev/line_laser_r", "/dev/line_laser_b",
    "/dev/line_laser_f"};
constexpr int kLinelaserFrequence = 10;
constexpr int kBandrateCode921600 = B921600;
constexpr int kBandrateCode115200 = B115200;
constexpr int kBandrateCode230400 = B230400;
constexpr uint8_t kLinelaserHearder = 0xA5;

constexpr uint8_t kLinelaserGetAddress = 0x60;
constexpr uint8_t kLinelaserGetParameter = 0x61;
constexpr uint8_t kLinelaserGetVersion = 0x62;
constexpr uint8_t kLinelaserStartScan = 0x63;
constexpr uint8_t kLinelaserStopScan = 0x64;
constexpr uint8_t kLinelaserRestartScan = 0x67;
constexpr uint8_t kLinelaserSetMode = 0x69;
constexpr uint8_t kLinelaserSetBias = 0xD9;

// constexpr uint8_t kLinelaserSetBaudrate = 0x68;
#endif