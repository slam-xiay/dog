#ifndef IMU_HPP
#define IMU_HPP
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/SetBool.h>
#include <Eigen/Core>
#include "serial.hpp"

// struct ImuData{
//   // uint32_t time;
//   Eigen::Vector3f raw_acc;
//   Eigen::Vector3f calibrated_acc;
//   Eigen::Vector3f raw_gyro0;
//   Eigen::Vector3f raw_gyro1;
//   Eigen::Vector3f calibrated_gyro0;
//   Eigen::Vector3f calibrated_gyro1;
//   Eigen::Vector3f aligned_gyro0;
//   Eigen::Vector3f aligned_gyro1;
//   Eigen::Vector3f raw_mag;
//   Eigen::Vector4f quaternion;
//   Eigen::Vector3f euler;
//   // Eigen::Vector3f lin_acc;
//   float pressure;
//   float altitude;
//   float temperature;
// };

struct ImuData{
  uint8_t time[4];
  float raw_acc[3];
  float calibrated_acc[3];
  float raw_gyro0[3];
  float raw_gyro1[3];
  float calibrated_gyro0[3];
  float calibrated_gyro1[3];
  float aligned_gyro0[3];
  float aligned_gyro1[3];
  float raw_mag[3];
  float calibrated_mag[3];
  float quaternion[4];
  float euler[3];
  // float lin_acc[3];
  // float pressure;
  // float altitude;
  float temperature;
};

struct ImuFrame {
  uint8_t header;
  uint8_t id[2];
  uint8_t command[2];
  uint8_t length[2];
  uint8_t data[156];
  // ImuData data;
  uint8_t lrc[2];
  uint8_t end[2];
};
  // uint8_t data[156];

class Imu : public Serial {
 public:
  Imu(std::shared_ptr<ros::NodeHandle> node_handle_ptr,
             const std::string &serial_port);
  ~Imu() = default;
  Imu(const Imu &) = default;
  Imu &operator=(const Imu &) = default;

  void ImuRxThread();

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
