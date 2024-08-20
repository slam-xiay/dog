#include "multilinelaser.hpp"// add tag 1
Multilinelaser::Multilinelaser(std::shared_ptr<BlackBoard> black_board_ptr):black_board_ptr_(black_board_ptr){  
    cloud_publisher_ = black_board_ptr_->GetNodeHandlePtr()->advertise<sensor_msgs::PointCloud>("multilinelaser_cloud", 1000);
    address_.sin_family = AF_INET;
    address_.sin_port = htons(2368);
    address_.sin_addr.s_addr = INADDR_ANY;
    fd_ = socket(PF_INET, SOCK_DGRAM, 0);
    LOG(ERROR)<<"Multilinelaser fd:("<<fd_<<")";
    int bind_res = bind(fd_, reinterpret_cast<sockaddr *>(&address_), sizeof(sockaddr));
    if (bind_res== -1) {
      LOG(ERROR)<<"Can not bind fd.";
      return;
    }else{
      LOG(ERROR)<<"Bind response:("<<bind_res<<")";
    }
    int fcntl_res = fcntl(fd_, F_SETFL, O_NONBLOCK | FASYNC);
    if (fcntl_res<0) {
      LOG(ERROR)<<"Cannot set block.";
      return;
    }else{
      LOG(ERROR)<<"fcntl response:("<<fcntl_res<<").";
    }
};
Multilinelaser::~Multilinelaser(){};

bool Multilinelaser::ReadFromIO(std::vector<uint8_t>& data){
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
  for (size_t i = 0; i < rx_len; i++) data.push_back(*(rx_buf + i));
  return rx_len > 0;
}

bool Multilinelaser::ParseData(std::vector<uint8_t>& data){
  cloud_.header.frame_id = "base_link";  // 没开里程计转换无法转换
  cloud_.channels.resize(1);
  cloud_.channels[0].name = "intensities";
  while (data.size() >= sizeof(MultilinelaserFrame)) {
    size_t drop = 0;
    // Eigen::Vector3d laser_link(kMultilinelaserToBaseTranslationX, 
    //   kMultilinelaserToBaseTranslationY, kMultilinelaserToBaseTranslationZ);
    while(data.size() >= sizeof(MultilinelaserFrame)){
      if(data[0] == kMultilinelaserHeader1&&data[1]==kMultilinelaserHeader2){
        break;
      }else{
        drop++;
        data.erase(data.begin());
      }
    }
    if(drop>0) LOG(ERROR)<<name_ <<" drop size:("<<drop<<").";
      if (data.size() < sizeof(MultilinelaserFrame))
        return false;
      else{
        std::vector<uint8_t> multiline_laser_data(data.begin(),
            data.end() + sizeof(MultilinelaserFrame));
        data.erase(data.begin(), data.begin() + sizeof(MultilinelaserFrame));
        MultilinelaserFrame *multiline_laser_frame = (MultilinelaserFrame*)multiline_laser_data.data();
        Time now = GetNow();
        for(size_t i = 0; i<8; i++){
          Time timestamp = now - (8-i)/kMultilinelaserFrequence;                    
        }

        for(auto&& block:multiline_laser_frame->block){
          double yaw = DegToRad(double(block.azimuth)/100);
          for(size_t i = 0;i<16;i++){
            double pitch = DegToRad(kMultilinelaserPitchOffset[i]);
            double range = double(block.channel[i].distance)*0.004;
            Eigen::Vector3d position = kMultilinelaserLink+Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                        Eigen::Vector3d(range, 0., 0.);
            geometry_msgs::Point32 pt;
            pt.x = position.x();
            pt.y = position.y();
            pt.z = position.z();
            cloud_.points.push_back(pt);
            cloud_.channels[0].values.push_back(block.channel[i].reflectivity);
          }
          if(yaw>2*kPI-0.01) {
            if(kPublishMultilinelaser) Publish();
            if(kBlackBoardMultilinelaser);
          }
          // LOG_EVERY_N(ERROR,30)<<"yaw:("<<yaw<<")";
        }      
        // LOG(ERROR)<<"time:("<<(uint32_t(multiline_laser_frame->timestamp[3])<<24
        //   |uint32_t(multiline_laser_frame->timestamp[2])<<16
        //   |uint32_t(multiline_laser_frame->timestamp[1])<<8
        //   |uint32_t(multiline_laser_frame->timestamp[0]))<<");";
        // LOG(ERROR)<<"year:("<<multiline_laser_frame->time[0]+1900<<");";
        // LOG(ERROR)<<"month:("<<int(multiline_laser_frame->time[1])<<");";
        // LOG(ERROR)<<"day:("<<int(multiline_laser_frame->time[2])<<");";
        // LOG(ERROR)<<"hour:("<<int(multiline_laser_frame->time[3])<<");";
        // LOG(ERROR)<<"minute:("<<int(multiline_laser_frame->time[4])<<");";
        // LOG(ERROR)<<"second:("<<int(multiline_laser_frame->time[5])<<");";
        // LOG(ERROR)<<"motor speed:("<<double(uint16_t(multiline_laser_frame->motor_speed[1])<<8|multiline_laser_frame->motor_speed[0])<<")";
      }
  }
  return true;
}

void Multilinelaser::Publish(){
  cloud_.header.stamp = ros::Time::now();
  cloud_publisher_.publish(cloud_);
  cloud_.points.clear();
  cloud_.channels[0].values.clear();
};
void Multilinelaser::MultilinelaserRxThread() {
  ros::Rate rate(kMultilinelaserFrequence);
  std::vector<uint8_t> rx_data;
  LOG(ERROR) << name_ << "thread started!";
  while (true) {
    if (is_running_ && ReadFromIO(rx_data)) ParseData(rx_data);
    rate.sleep();
  }
}