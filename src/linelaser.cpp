#include "linelaser.hpp"

Linelaser::Linelaser(std::shared_ptr<ros::NodeHandle> node_handle_ptr,
                     const std::string &serial_port)
    : node_handle_ptr_(node_handle_ptr),
      Serial(serial_port, kBandrateCode921600) {
  name_ = serial_port.substr(5);
  publisher_ = node_handle_ptr_->advertise<sensor_msgs::Range>(name_, 1);
  switch_ = node_handle_ptr_->advertiseService(
      name_, &Linelaser::SwitchCallBack, this);
};

void Linelaser::LinelaserRxThread() {
  ros::Rate rate(kLinelaserFrequence);
  std::vector<uint8_t> rx_data;
  LOG(ERROR) << name_ << " thread started!";
  while (true) {
    if (is_running_ && ReadFromIO(rx_data)) ParseData(rx_data);
    rate.sleep();
  }
};

bool Linelaser::ParseData(std::vector<uint8_t> &data) {
  // LOG(ERROR) << name_ << " data size:(" << data.size() << ").";

  // if (data.size() >= 4) {
  //   static auto last = ros::Time::now();
  //   auto current = ros::Time::now();
  //   double duration = (current - last).toSec();
  //   LOG(ERROR) << name_ << " rx length(" << data.size()
  //              << "),duration from last(" << duration << ");";
  //   last = current;
  // }
  // data.clear();
  while (data.size() >= 322) {
    size_t drop = 0;
    while (data.size() >= sizeof(LinelaserFrame) &&
           data[0] != kLinelaserHearder && data[1] != kLinelaserHearder &&
           data[2] != kLinelaserHearder && data[3] != kLinelaserHearder) {
      data.erase(data.begin());
      drop++;
    }
    LOG(ERROR) << "Drop size:(" << drop << ").";
    if (data.size() < sizeof(LinelaserFrame)) break;
    int length = int(uint16_t(data[7]) << 8 | uint16_t(data[6]));
    LOG(ERROR) << "Data length:(" << length << ").";

    data.erase(data.begin(), data.begin() + 4);
  }
  // data.clear();
  //   while (data.size() >= 0 && data[0] != kLinelaserFrameHeader)
  //     data.erase(data.begin());
  //   if (data.size() < sizeof(LinelaserFrame)) break;
  //   std::vector<uint8_t> Linelaser_data(data.begin(),
  //                                       data.end() + sizeof(LinelaserFrame));
  //   data.erase(data.begin(), data.begin() + sizeof(LinelaserFrame));
  //   LinelaserFrame *Linelaser_frame = (LinelaserFrame
  //   *)Linelaser_data.data();
  //   uint8_t summary = Linelaser_frame->highbits + Linelaser_frame->lowbits +
  //                     Linelaser_frame->header;
  //   if (summary != Linelaser_frame->summary) {
  //     LOG(ERROR) << name_ << " crc check error!";
  //     continue;
  //   }
  //   uint16_t data_byte = uint16_t(Linelaser_frame->highbits) << 8 |
  //                        uint16_t(Linelaser_frame->lowbits);
  //   if (data_byte == 0xFFFF) {
  //     LOG(ERROR) << name_ << " timeout.";
  //     continue;
  //   }
  //   if (data_byte == 0xEEEE || data_byte == 0xFFFD) {
  //     // LOG(ERROR) << name_ << " invalid.";
  //     continue;
  //   }
  //   if (data_byte == 0xFFFE) {
  //     LOG(ERROR) << name_ << " co frequency interference.";
  //     continue;
  //   }
  //   float range = float(int(data_byte)) / 1000;
  //   if (range < 0.05 || range > 1.5) {
  //     // LOG(ERROR) << std::hex << std::uppercase << "(" << data_byte <<
  //     ").";
  //     // LOG(ERROR) << name_ << " out of range:(" << range << ").";
  //     // range = std::numeric_limits<float>::quiet_NaN();
  //   } else {
  //     LOG(ERROR) << name_ << " publish range:(" << range << ").";
  //     Publish(range);
  //   }
  // }
};

bool Linelaser::SetBandrate(const int &bandrate_code) {
  // 针对不同的波特率分别连接串口，发送停机，改波特率的命令。
};

bool Linelaser::SetStartCommand() {
  bool is_start = false;
  std::vector<uint8_t> rx_data;
  while (!is_start) {
    usleep(10000);
    WriteToIO(kLinelaserStartCommand);
    usleep(10000);
    ReadFromIO(rx_data);
    usleep(10000);
    while (rx_data.size() >= 4) {
      if (rx_data[0] == kLinelaserHearder && rx_data[1] == kLinelaserHearder &&
          rx_data[2] == kLinelaserHearder && rx_data[3] == kLinelaserHearder) {
        printf("\n");
        printf("Data size has headers:(%lu)", rx_data.size());
        printf("\n");
        for (auto &&byte : rx_data) {
          printf("%02x ", byte);
        }
        printf("\n");
        rx_data.clear();
      } else {
        printf("%02x ", rx_data[0]);
        rx_data.erase(rx_data.begin());
      }
    }
    printf("Rx size:%ld\n", rx_data.size());
    usleep(10000);
  }
  return is_start;
};

void Linelaser::Publish(const float &range) {
  static auto last = ros::Time::now();
  sensor_msgs::Range msg;
  msg.header.frame_id = name_;
  msg.header.stamp = ros::Time::now();
  msg.range = range;
  publisher_.publish(msg);
  last = msg.header.stamp;
}

void Linelaser::SetRunning(const bool &running) {
  if (running) {
    Close();
    Open();
    SetStartCommand();
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
    WriteToIO(kLinelaserStopCommand);
    Close();
    // WriteToIO({kLinelaserStopCode});
  }
  is_running_ = running;
}

bool Linelaser::SwitchCallBack(std_srvs::SetBool::Request &req,
                               std_srvs::SetBool::Response &res) {
  SetRunning(req.data);
  return true;
};
