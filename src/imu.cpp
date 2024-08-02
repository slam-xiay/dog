#include "imu.hpp"
Imu::Imu(std::shared_ptr<ros::NodeHandle> node_handle_ptr, const std::string &serial_port)
    : node_handle_ptr_(node_handle_ptr),
      Serial(serial_port) {
  name_ = serial_port.substr(5);
  publisher_ = node_handle_ptr_->advertise<sensor_msgs::Range>(name_, 1);
  switch_ = node_handle_ptr_->advertiseService(
      name_, &Imu::SwitchCallBack, this);
  Close();
  Open(kBandrateCode921600);
  is_running_=true;
};

void Imu::ImuRxThread() {
  ros::Rate rate(kLinelaserFrequence);
  std::vector<uint8_t> rx_data;
  LOG(ERROR) << name_ << " thread started!";
  while (true) {
    if (is_running_ && ReadFromIO(rx_data)) ParseData(rx_data);
    rate.sleep();
  }
};

bool Imu::ParseData(std::vector<uint8_t> &data) {
  auto crc_check = [](const std::vector<uint8_t> &data) {
    if(data.size()<sizeof(ImuFrame)) return false;
    uint16_t summary = 0;
    for(size_t i = 1;i<=162;i++) summary += data[i];
    uint16_t crc = uint16_t(data[164])<< 8 | uint16_t(data[163]);
    return summary==crc;
  };

  // LOG(ERROR) << name_ << " ImuData size:(" << sizeof(ImuData) << ").";
  while (data.size() >= sizeof(ImuFrame)) {
    size_t drop = 0;
    while(data.size() >= sizeof(ImuFrame)){
      if(data[0] == kImuHeader1 && data[1]==kImuAddress1 && data[2]==kImuAddress2){
        break;
      }else{
        drop++;
        data.erase(data.begin());
      }
    }
    LOG(ERROR) << "Drop size:(" << drop << "),data size:("<<data.size()<<").";
    if (data.size() < sizeof(ImuFrame)) break;
    int length = int(uint16_t(data[6]) << 8 | uint16_t(data[5]));
    LOG(ERROR) << "Data length:(" << length << ").";
    if (data.size() < sizeof(ImuFrame)) break;
    std::vector<uint8_t> imu_data(data.begin(),data.begin() + sizeof(ImuFrame));
    data.erase(data.begin(), data.begin() + sizeof(ImuFrame));
    LOG(ERROR)<<"Imu data size:("<<imu_data.size()<<"),ImuFrame size:("<<sizeof(ImuFrame)<<"),data size:("<<sizeof(ImuData)<<")";
    ImuFrame *imu_frame = (ImuFrame*)imu_data.data();
    if(crc_check(data)) LOG(ERROR) << "CRC check successful.";
    else{
      // LOG(ERROR)<<"CRC check failed.";
      continue;
    } 
 
    std::vector<uint8_t> roll_vector(imu_data.begin()+147,imu_data.begin()+151);
    float roll = *(float*)roll_vector.data();
    LOG(ERROR)<<"roll size:("<<roll_vector.size()<<"),roll:("<<roll<<").";
    std::vector<uint8_t> pitch_vector(imu_data.begin()+151,imu_data.begin()+155);
    float pitch = *(float*)pitch_vector.data();
    LOG(ERROR)<<"pitch size:("<<pitch_vector.size()<<"),pitch:("<<pitch<<").";  
    std::vector<uint8_t> yaw_vector(imu_data.begin()+155,imu_data.begin()+159);
    float yaw = *(float*)yaw_vector.data();
    LOG(ERROR)<<"yaw size:("<<yaw_vector.size()<<"),yaw:("<<yaw<<").";
    std::vector<uint8_t> temperature_vector(imu_data.begin()+159,imu_data.begin()+163);
    float temperature = *(float*)temperature_vector.data();
    LOG(ERROR)<<"temperature size:("<<temperature_vector.size()<<"),temperature:("<<temperature<<").";
    // LOG(ERROR)<<"temperature:("<<temperature<<")";
    // LOG(ERROR)<<"pitch:("<<pitch<<"),roll:("<<roll<<"),yaw:("<<yaw<<")";
    // Eigen::Vector3f euler(imu_frame->data.euler[0],imu_frame->data.euler[1],imu_frame->data.euler[2]);
    // LOG(ERROR)<<"roll:("<<euler[0]<<"),pitch:("<<euler[1]<<"),yaw:("<<euler[2]<<")";
    // LOG(ERROR)<<"tempature:("<<imu_frame->data.temperature<<")";
    // LOG(ERROR)<<"imu_data back:()";
    // printf("back:(%.2x================%.2x)",imu_data[165],imu_data[166]);
  }

//   // if (data.size() >= 4) {
//   //   static auto last = ros::Time::now();
//   //   auto current = ros::Time::now();
//   //   double duration = (current - last).toSec();
//   //   LOG(ERROR) << name_ << " rx length(" << data.size()
//   //              << "),duration from last(" << duration << ");";
//   //   last = current;
//   // }
//   // data.clear();
//   while (data.size() >= 322) {
//     size_t drop = 0;
//     while (data.size() >= sizeof(LinelaserFrame) &&
//            data[0] != kLinelaserHearder && data[1] != kLinelaserHearder &&
//            data[2] != kLinelaserHearder && data[3] != kLinelaserHearder) {
//       data.erase(data.begin());
//       drop++;
//     }
//     LOG(ERROR) << "Drop size:(" << drop << ").";
//     if (data.size() < sizeof(LinelaserFrame)) break;
//     int length = int(uint16_t(data[7]) << 8 | uint16_t(data[6]));
//     LOG(ERROR) << "Data length:(" << length << ").";

//     data.erase(data.begin(), data.begin() + 4);
//   }
//   return true;
//   // data.clear();
//   //   while (data.size() >= 0 && data[0] != kLinelaserFrameHeader)
//   //     data.erase(data.begin());
//   //   if (data.size() < sizeof(LinelaserFrame)) break;
//   //   std::vector<uint8_t> Linelaser_data(data.begin(),
//   //                                       data.end() + sizeof(LinelaserFrame));
//   //   data.erase(data.begin(), data.begin() + sizeof(LinelaserFrame));
//   //   LinelaserFrame *Linelaser_frame = (LinelaserFrame
//   //   *)Linelaser_data.data();
//   //   uint8_t summary = Linelaser_frame->highbits + Linelaser_frame->lowbits +
//   //                     Linelaser_frame->header;
//   //   if (summary != Linelaser_frame->summary) {
//   //     LOG(ERROR) << name_ << " crc check error!";
//   //     continue;
//   //   }
//   //   uint16_t data_byte = uint16_t(Linelaser_frame->highbits) << 8 |
//   //                        uint16_t(Linelaser_frame->lowbits);
//   //   if (data_byte == 0xFFFF) {
//   //     LOG(ERROR) << name_ << " timeout.";
//   //     continue;
//   //   }
//   //   if (data_byte == 0xEEEE || data_byte == 0xFFFD) {
//   //     // LOG(ERROR) << name_ << " invalid.";
//   //     continue;
//   //   }
//   //   if (data_byte == 0xFFFE) {
//   //     LOG(ERROR) << name_ << " co frequency interference.";
//   //     continue;
//   //   }
//   //   float range = float(int(data_byte)) / 1000;
//   //   if (range < 0.05 || range > 1.5) {
//   //     // LOG(ERROR) << std::hex << std::uppercase << "(" << data_byte <<
//   //     ").";
//   //     // LOG(ERROR) << name_ << " out of range:(" << range << ").";
//   //     // range = std::numeric_limits<float>::quiet_NaN();
//   //   } else {
//   //     LOG(ERROR) << name_ << " publish range:(" << range << ").";
//   //     Publish(range);
//   //   }
//   // }
};

// bool Imu::SetBandrate(const int &bandrate_code) {
//   // 针对不同的波特率分别连接串口，发送停机，改波特率的命令。
//   return true;
// };

// bool Imu::SetStartCommand() {
//   bool is_start = false;
//   std::vector<uint8_t> rx_data;
//   while (!is_start) {
//     usleep(10000);
//     WriteToIO(kLinelaserStartCommand);
//     usleep(10000);
//     ReadFromIO(rx_data);
//     usleep(10000);
//     while (rx_data.size() >= 4) {
//       if (rx_data[0] == kLinelaserHearder && rx_data[1] == kLinelaserHearder &&
//           rx_data[2] == kLinelaserHearder && rx_data[3] == kLinelaserHearder) {
//         printf("\n");
//         printf("Data size has headers:(%lu)", rx_data.size());
//         printf("\n");
//         for (auto &&byte : rx_data) {
//           printf("%02x ", byte);
//         }
//         printf("\n");
//         rx_data.clear();
//       } else {
//         printf("%02x ", rx_data[0]);
//         rx_data.erase(rx_data.begin());
//       }
//     }
//     printf("Rx size:%ld\n", rx_data.size());
//     usleep(10000);
//   }
//   return is_start;
// };

// void Imu::Publish(const float &range) {
//   static auto last = ros::Time::now();
//   sensor_msgs::Range msg;
//   msg.header.frame_id = name_;
//   msg.header.stamp = ros::Time::now();
//   msg.range = range;
//   publisher_.publish(msg);
//   last = msg.header.stamp;
// }

void Imu::SetRunning(const bool &running) {
  if (running) {
    Close();
    Open(kBandrateCode115200);
    // SetStartCommand();
    // sleep(1);
    // WriteToIO(kLinelaserSet230400);
    // WriteToIO(kLinelaserGetAddressCommand);
    // WriteToIO(kLinelaserGetParameterCommand);
    // WriteToIO(kLinelaserGetVersionCommand);
    // WriteToIO(kLinelaserStopCommand);
    // WriteToIO(kLinelaserSet921600);

    // WriteToIO(kLinelaserStartCommand);
    // // sleep(1);
    // WriteToIO(kLinelaserSet921600);
    // sleep(1);
    // WriteToIO({kLinelaserGetAddress});
    // WriteToIO({kLinelaserGetVersion});
  } else {
    // WriteToIO(kLinelaserStopCommand);
    Close();
    // WriteToIO({kLinelaserStopCode});
  }
  is_running_ = running;
}

bool Imu::SwitchCallBack(std_srvs::SetBool::Request &req,
                               std_srvs::SetBool::Response &res) {
  SetRunning(req.data);
  return true;
};
