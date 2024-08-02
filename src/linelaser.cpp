#include "linelaser.hpp"

Linelaser::Linelaser(std::shared_ptr<ros::NodeHandle> node_handle_ptr,
                     const std::string &serial_port)
    : node_handle_ptr_(node_handle_ptr),
      Serial(serial_port) {
  name_ = serial_port.substr(5);
  publisher_ = node_handle_ptr_->advertise<sensor_msgs::Range>(name_, 1);
  switch_ = node_handle_ptr_->advertiseService(
      name_, &Linelaser::SwitchCallBack, this);
};

void Linelaser::LinelaserRxThread() {
  ros::Rate rate(kLinelaserFrequence);
  std::vector<uint8_t> rx_data;
  while (true) {
    // LOG_EVERY_N(ERROR,1)<<name_<<" loop is running:("<<is_running_<<"),is_start:("<<is_start_<<"),fd:("<<fd_<<");";
    if (is_running_ && ReadFromIO(rx_data)) ParseData(rx_data);
    rate.sleep();
  }
};

void Linelaser::ParseData(std::vector<uint8_t> &data) {
  auto get_average_range = [&](const std::vector<uint8_t> &data) {
    float average = 0.f;
    if (data.size() != sizeof(LinelaserDataFrame)) return average;
    for (size_t i = 10; i < data.size() - 1; i += 2) {
      float range = float((data[i] << 8 | data[i + 1]) & 0x01FF) / 1000;
      average += range;
    }
    LOG_EVERY_N(ERROR,100) <<name_<< "average range:(" << average / 160 << ").";
    return (average / 160);
  };

  auto get_frame_crc = [&](const std::vector<uint8_t> &data) {
    uint8_t summary = 0;
    if (data.size() != sizeof(LinelaserDataFrame)) return summary;
    for (size_t i = 4; i < sizeof(LinelaserDataFrame) - 1; i++){
      summary += data[i];
      // printf("i:(%lu):(%02x) ", i,data[i]);
    }
    // printf("\n=====get_frame_crc=====\n");
    // for (auto&& byte : data) {
    //   printf("%02x ", byte);
    // }
    // printf("\n=====get_frame_crc=====(%02x)(%02x)\n",summary,data[sizeof(LinelaserDataFrame) - 1]);
    return summary;
  };

  auto get_command_crc = [&](const std::vector<uint8_t> &data) {
    uint8_t summary = 0;
    if (data.size() != sizeof(LinelaserCommandFrame)) return summary;
    for (size_t i = 4; i < sizeof(LinelaserCommandFrame) - 1; i++)
      summary += data[i];
    return summary;
  };

  // LOG(ERROR) << "Data size before while: (" << data.size() << ").";
  while (data.size() >= sizeof(LinelaserCommandFrame)) {
    size_t drop = 0;
    while (data.size() >= sizeof(LinelaserCommandFrame)) {
      if (data[0] == kLinelaserHearder && data[1] == kLinelaserHearder &&
          data[2] == kLinelaserHearder && data[3] == kLinelaserHearder){
        // LOG(ERROR)<<"Find header size:("<<data.size()<<").";
        break;
      }
      else{
        drop++;
        // printf("drop:(%lu):(%02x) ", drop,data[0]);
        // printf("%02x ", data[0]);
        data.erase(data.begin());
      }
    }
    if(drop>0) LOG(ERROR)<<name_ <<" drop size:("<<drop<<").";
    if (data.size() < sizeof(LinelaserCommandFrame))
      return;
    else {
      size_t length = size_t(uint16_t(data[7]) << 8 | uint16_t(data[6]));
      // LOG(ERROR) << "length:(" << length << "),size:(" << data.size() << ").";
      switch (length) {
        case kLinelaserCommandLength:
          if (data.size() >= sizeof(LinelaserCommandFrame)) {
            std::vector<uint8_t> linelaser_command(
                data.begin(), data.begin() + sizeof(LinelaserCommandFrame));
            data.erase(data.begin(), data.begin() + sizeof(LinelaserCommandFrame));
            if (get_command_crc(linelaser_command) != linelaser_command.back()) {
              //  printf("get_frame_crc:(%02x %02x).\n", get_frame_crc(linelaser_frame) ,linelaser_frame.back());
              // LOG(ERROR)<<std::showbase<<std::hex<<"summary:("<<(int)(linelaser_frame.back())<<"),crc:("<<(int)(get_frame_crc(linelaser_frame))<<")";
              LOG(ERROR) << name_ << " command crc error.";
            } else {
              LOG(ERROR) << "Receive command ======= ("<<std::showbase<<std::hex<<int(data[5])<<").";
              switch (data[5]){
                case 0x63:
                  LOG(ERROR) <<name_<< " Receive start command response.";
                  is_start_ = true;
                  break;
                case 0x64:
                  LOG(ERROR) <<name_<< " Receive stop command response.";
                  is_start_ = false;
                  break;
                case 0x68:
                  LOG(ERROR) <<name_<<" Receive set bandrate response.";
                  is_right_bandrate_ = true;
                  break;
                default:
                  LOG(ERROR) <<name_<< " Receive other command response.";
                  break;
              }
            }           
            break;
          } else {
            return;
          }
        case kLinelaserDataLength:
          if (data.size() >= sizeof(LinelaserDataFrame)) {
            is_start_ = true;
            // LOG(ERROR)<<"Find header data size:("<<data.size()<<"),length:("<<length<<"),frame size:("<<sizeof(LinelaserDataFrame)<<").";
            std::vector<uint8_t> linelaser_frame(
                data.begin(), data.begin() + sizeof(LinelaserDataFrame));
            data.erase(data.begin(), data.begin() + sizeof(LinelaserDataFrame));
            if (get_frame_crc(linelaser_frame) != linelaser_frame.back()) {
              //  printf("get_frame_crc:(%02x %02x).\n", get_frame_crc(linelaser_frame) ,linelaser_frame.back());
              // LOG(ERROR)<<std::showbase<<std::hex<<"summary:("<<(int)(linelaser_frame.back())<<"),crc:("<<(int)(get_frame_crc(linelaser_frame))<<")";
              LOG(ERROR) << name_ << " frame crc error.";
            } else {
              // if(name_ == "/dev/line_laser_f"){
              //   data[]
              // }
              Publish(name_,get_average_range(linelaser_frame));
              // if(name_ != "/dev/line_laser_f")
              //   Publish(name_,get_average_range(linelaser_frame));
              // else if(){

              // }
            }
            break;
          } else {
            return;
          }
        default:
          LOG(ERROR) << "Receive error frame,length:(" << length << ").";
          data.clear();
          return;
      }
    }
  }
};
// void Linelaser::Publish(const float &range) {
//   static auto last = ros::Time::now();
//   sensor_msgs::Range msg;
//   msg.header.frame_id = name_;
//   msg.header.stamp = ros::Time::now();
//   msg.range = range;
//   publisher_.publish(msg);
//   last = msg.header.stamp;
// }

void Linelaser::Publish(const std::string& name,const float &range) {
  static auto last = ros::Time::now();
  sensor_msgs::Range msg;
  msg.header.frame_id = name;
  msg.header.stamp = ros::Time::now();
  msg.range = range;
  publisher_.publish(msg);
  last = msg.header.stamp;
}

bool Linelaser::SetRunning(const bool &running) {
  ros::Rate rate(kLinelaserFrequence);
  LOG(ERROR) << "SetRunning:(" << running << "),is_start:(" << is_start_ << ")";
  if (running) {
    Close();
    Open(kBandrateCode921600);
    is_running_ = true;
    auto start = ros::Time::now();
    while ((ros::Time::now() - start).toSec() < 1) {
      if (is_start_) {
        LOG(ERROR) << "Set running true successful.";
        return true;
      } else {
        if (WriteToIO(kLinelaserStartCommand))
          LOG(ERROR) << name_<<" Send kLinelaserStartCommand successful.";
        else
          LOG(ERROR) << name_<<" Send kLinelaserStartCommand failure.";
      }
      rate.sleep();
    }
    LOG(ERROR) << "Set running true time out.";
    return false;
  } else {
    auto start = ros::Time::now();
    while ((ros::Time::now() - start).toSec() < 1) {
      if (!is_start_) {
        LOG(ERROR) << "Set running false successful.";
        return true;
      } else {
        if (WriteToIO(kLinelaserStopCommand))
          LOG(ERROR) <<name_<< " Send kLinelaserStopCommand successful.";
        else
          LOG(ERROR) <<name_<< " Send kLinelaserStopCommand failure.";
      }
      usleep(10000);
    }
    LOG(ERROR) << "Set running false time out.";
    Close();
    is_running_ = false;
    return true;
  }
}

bool Linelaser::SwitchCallBack(std_srvs::SetBool::Request &req,
                               std_srvs::SetBool::Response &res) {
  if (SetRunning(req.data)) {
    res.success = true;
    return true;
  } else {
    res.success = false;
    return false;
  }
};