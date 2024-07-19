#ifndef ULTRASONIC_HPP
#define ULTRASONIC_HPP
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/SetBool.h>

#include "serial.hpp"

struct UltrasonicFrame {
  uint8_t header;
  uint8_t highbits;
  uint8_t lowbits;
  uint8_t summary;
};

class Ultrasonic : public Serial {
 public:
  Ultrasonic(std::shared_ptr<ros::NodeHandle> node_handle_ptr,
             const std::string &serial_port);
  ~Ultrasonic() = default;
  Ultrasonic(const Ultrasonic &) = default;
  Ultrasonic &operator=(const Ultrasonic &) = default;

  void UltrasonicRxThread();

  bool ParseData(std::vector<uint8_t> &data);

  void Publish(const float &range);

  void SetRunning(const bool &running);

  bool SwitchCallBack(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res);

 private:
  std::shared_ptr<ros::NodeHandle> node_handle_ptr_;
  ros::Publisher publisher_;
  ros::ServiceServer switch_;
  std::string name_;
  bool is_running_{false};
};

#endif
