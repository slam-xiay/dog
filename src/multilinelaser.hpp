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
#include <netinet/in.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Core>
#include <Eigen/Dense>
struct ChannelFrame{
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t reserve;
};

struct BlockFrame{
  uint16_t azimuth;
  ChannelFrame channel[16];
};

struct MultilinelaserFrame {
  uint8_t header1;
  uint8_t header2;
  uint8_t version_major;
  uint8_t version_minor;
  uint8_t reserve1;
  uint8_t reserve2;
  uint8_t channel_count;// 0x10(16)
  uint8_t block_count; //0x08(8)
  uint8_t reserve3;
  uint8_t distance_unit;//0x04(4mm)
  uint8_t return_count;//0x02(2)
  uint8_t reserve4;
  BlockFrame block[8];
  uint8_t tail_reserve[10];
  uint8_t return_mode;
  uint8_t motor_speed[2];//10hz*60 1分钟600转
  uint8_t time[6];
  uint8_t timestamp[4];
  uint8_t factory_information;
  uint8_t udp_sequence[4];
};

class Multilinelaser
{
public:
  Multilinelaser(std::shared_ptr<ros::NodeHandle> node_handle_ptr);
  ~Multilinelaser();
  Multilinelaser(const Multilinelaser &) = delete;
  Multilinelaser &operator=(const Multilinelaser &) = delete;
  void MultilinelaserRxThread();
  bool ReadFromIO(std::vector<uint8_t>& data);
  bool ParseData(std::vector<uint8_t> &data);
  void Publish();
private:
  
  int fd_{-1};
  sockaddr_in address_;
  std::shared_ptr<ros::NodeHandle> node_handle_ptr_;
  ros::Publisher publisher_;
  ros::ServiceServer switch_;
  std::string name_;
  bool is_running_{true};
  sensor_msgs::PointCloud cloud_;
  ros::Publisher cloud_publisher_;
};

#endif 
