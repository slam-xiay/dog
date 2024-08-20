#ifndef DRIVERS_IMU_HPP
#define DRIVERS_IMU_HPP
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/SetBool.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>

#include "serial.hpp"
#include "blackboard.hpp"

#pragma pack(1)
struct ImuHeader{
  uint8_t header;
  uint8_t id[2];
  uint8_t command[2];
  uint16_t length;
};

struct ImuTail{
  uint8_t lrc[2];
  uint8_t end[2];
};

struct ImuFrame{
  uint8_t header;
  uint8_t id[2];
  uint8_t command[2];
  uint16_t length;
  uint8_t time[4];
  float raw_acc[3];
  float corrected_acc[3];
  float raw_gyro0[3];
  float raw_gyro1[3];
  float corrected_gyro0[3];
  float corrected_gyro1[3];
  float aligned_gyro0[3];
  float aligned_gyro1[3];
  float raw_mag[3];
  float corrected_mag[3];
  float quaternion[4];
  float euler[3];
  float temperature;
  uint8_t lrc[2];
  uint8_t end[2];
};

const size_t kImuFrameSize = sizeof(ImuFrame);
const size_t kImuContentSize = kImuFrameSize - sizeof(ImuHeader) - sizeof(ImuTail);
#pragma pack()
class Imu : public Serial {
 public:
  Imu(std::shared_ptr<BlackBoard> black_board_ptr,
             const std::string &serial_port);
  ~Imu();
  Imu(const Imu &) = default;
  Imu &operator=(const Imu &) = default;

  void ImuRxThread();

  bool ParseData(std::vector<uint8_t> &data);

  void CachePry(const ImuFrame *imu_frame);

  void Publish(const ImuFrame *imu_frame);

  void SetRunning(const bool &running);

  bool SwitchCallBack(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res);

 private:
  std::shared_ptr<ros::NodeHandle> node_handle_ptr_;
  std::shared_ptr<BlackBoard> black_board_ptr_;
  ros::Publisher publisher_;
  ros::ServiceServer switch_;
  std::string name_;
  bool is_running_{false};
  std::vector<ros::Time> times_;
  
};

#endif
