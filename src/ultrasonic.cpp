#include "ultrasonic.hpp"

Ultrasonic::Ultrasonic(std::shared_ptr<ros::NodeHandle> node_handle_ptr,
                       const std::string &serial_port)
    : node_handle_ptr_(node_handle_ptr),
      Serial(serial_port) {
  name_ = serial_port.substr(5);
  publisher_ = node_handle_ptr_->advertise<sensor_msgs::Range>(name_, 1);
  switch_ = node_handle_ptr_->advertiseService(
      name_, &Ultrasonic::SwitchCallBack, this);
};
void Ultrasonic::UltrasonicRxThread() {
  ros::Rate rate(kUltrasonicFrequence);
  std::vector<uint8_t> rx_data;
  LOG(ERROR) << name_ << " thread started!";
  while (true) {
    if (is_running_ && ReadFromIO(rx_data)) ParseData(rx_data);
    rate.sleep();
  }
};
void Ultrasonic::ParseData(std::vector<uint8_t> &data) {
  LOG(ERROR)<<"size:("<<data.size()<<").";
  if (data.size() >= 1) {
    static auto last = ros::Time::now();
    auto current = ros::Time::now();
    double duration = (current - last).toSec();
    if (duration > 1e-4)
      LOG(ERROR) << name_ << " rx length(" << data.size()
                 << "),duration from last(" << duration << ");";
    last = current;
  }

  while (data.size() >= sizeof(UltrasonicFrame)) {
    while (data.size() >= 0 && data[0] != kUltrasonicFrameHeader)
      data.erase(data.begin());
    if (data.size() < sizeof(UltrasonicFrame)) break;
    std::vector<uint8_t> ultrasonic_data(data.begin(),
                                         data.end() + sizeof(UltrasonicFrame));
    data.erase(data.begin(), data.begin() + sizeof(UltrasonicFrame));
    UltrasonicFrame *ultrasonic_frame =
        (UltrasonicFrame *)ultrasonic_data.data();
    uint8_t summary = ultrasonic_frame->highbits + ultrasonic_frame->lowbits +
                      ultrasonic_frame->header;
    if (summary != ultrasonic_frame->summary) {
      LOG(ERROR) << name_ << " crc check error!";
      continue;
    }
    uint16_t data_byte = uint16_t(ultrasonic_frame->highbits) << 8 |
                         uint16_t(ultrasonic_frame->lowbits);
    if (data_byte == 0xFFFF) {
      LOG(ERROR) << name_ << " timeout.";
      continue;
    }
    if (data_byte == 0xEEEE || data_byte == 0xFFFD) {
      LOG(ERROR) << name_ << " invalid.";
      continue;
    }
    if (data_byte == 0xFFFE) {
      LOG(ERROR) << name_ << " co frequency interference.";
      continue;
    }
    float range = float(int(data_byte)) / 1000;
    if (range < 0.05 || range > 1.5) {
      // LOG(ERROR) << std::hex << std::uppercase << "(" << data_byte << ").";
      LOG(ERROR) << name_ << " out of range:(" << range << ").";
      // range = std::numeric_limits<float>::quiet_NaN();
    } else {
      LOG(ERROR) << name_ << " publish range:(" << range << ").";
      Publish(name_,range);
    }
  }
};

// bool Ultrasonic::ParseData(const char *buffer, size_t length) {
//   if (length >= 4) {
//     static auto last = ros::Time::now();
//     auto current = ros::Time::now();
//     double duration = (current - last).toSec();
//     LOG(ERROR) << name_ << " rx length(" << int(length)
//                << "),duration from last(" << duration << ");";
//     last = current;
//   }

//   // std::vector<char> data;
//   // for (int i = 0; i < length; i++) data.push_back(*(buffer + i));
//   // if (data.size() < sizeof(UltrasonicFrame) ||
//   //     data.size() >= 10 * sizeof(UltrasonicFrame)) {
//   //   LOG(ERROR) << "Ultrasonic size error:(" << data.size() << ")bytes;";
//   //   return false;
//   // }
//   // while (data.size() >= sizeof(UltrasonicFrame)) {
//   //   while (data.size() >= sizeof(UltrasonicFrame)) {
//   //     if (data[0] != kUltrasonicFrameHeader) {
//   //       data.erase(data.begin());
//   //       LOG(ERROR) << "Delete error ultrasonic(" << name_ << ") data.";
//   //     } else {
//   //       break;
//   //     }
//   //   }
//   //   std::vector<char> ultrasonic_data(data.begin(),
//   //                                     data.end() +
//   sizeof(UltrasonicFrame));
//   //   data.erase(data.begin(), data.begin() + sizeof(UltrasonicFrame));
//   //   UltrasonicFrame *ultrasonic_frame =
//   //       (UltrasonicFrame *)ultrasonic_data.data();
//   //   uint16_t data_byte =
//   //       ultrasonic_frame->highbits << 8 | ultrasonic_frame->lowbits;
//   //   if (data_byte == 0xFFFF) LOG(ERROR) << name_ << " timeout.";
//   //   if (data_byte == 0xEEEE) LOG(ERROR) << name_ << " data invalid.";
//   //   LOG(ERROR) << "Data byte: (" << std::hex << data_byte << ").";
//   //   float range = float(data_byte) / 1000;
//   //   LOG(ERROR) << name_ << " range:(" << range << ").";
//   //   if (range < 0.05 || range > 1.5) {
//   //     // LOG(ERROR) << name_ << " out of range:(" << range << ").";
//   //     range = std::numeric_limits<float>::quiet_NaN();
//   //   }

//   //   Publish(range);
//   // }
//   return true;
// };
void Ultrasonic::Publish(const std::string& name,const float &range) {
  static auto last = ros::Time::now();
  sensor_msgs::Range msg;
  msg.header.frame_id = name;
  msg.header.stamp = ros::Time::now();
  msg.range = range;
  publisher_.publish(msg);
}

bool Ultrasonic::SetRunning(const bool &running) {
  if (running) {
    Close();
    Open(kBandrateCode115200);
    WriteToIO({0xAA});
    is_running_ = true;
  } else {
    Close();
    is_running_ = false;
  }
  return true;
}

bool Ultrasonic::SwitchCallBack(std_srvs::SetBool::Request &req,
                                std_srvs::SetBool::Response &res) {
  if (SetRunning(req.data)) {
    res.success = true;
    return true;
  } else {
    res.success = false;
    return false;
  }
};

// void Ultrasonic::