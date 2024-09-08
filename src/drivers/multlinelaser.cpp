#include "multilinelaser.hpp"// add tag 1
Multilinelaser::Multilinelaser(std::shared_ptr<BlackBoard> black_board_ptr):black_board_ptr_(black_board_ptr){  
    cloud_publisher_ = black_board_ptr_->GetNodeHandlePtr()->advertise<sensor_msgs::PointCloud>("multilinelaser_cloud", 1000);
    intensity_cloud_publisher_ = black_board_ptr_->GetNodeHandlePtr()->advertise<sensor_msgs::PointCloud>("intensity_cloud", 1000);
    address_.sin_family = AF_INET;
    address_.sin_port = htons(2368);
    address_.sin_addr.s_addr = INADDR_ANY;
    fd_ = socket(PF_INET, SOCK_DGRAM, 0);

    cloud_.header.frame_id = "base_link";
    cloud_.channels.resize(1);
    cloud_.channels[0].name = "intensities";

    intensity_cloud_.header.frame_id = "base_link";
    intensity_cloud_.channels.resize(1);
    intensity_cloud_.channels[0].name = "intensities";

    // line_clouds_.resize(16);
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

  while (data.size() >= sizeof(MultilinelaserFrame)) {
    size_t drop = 0;
    while(data.size() >= sizeof(MultilinelaserFrame)){
      if(data[0] == kMultilinelaserHeader1&&data[1]==kMultilinelaserHeader2){
        break;
      }else{
        drop++;
        data.erase(data.begin());
      }
    }
    if(drop>0) LOG(ERROR)<<name_ <<" drop size:("<<drop<<").";
      if (data.size() < sizeof(MultilinelaserFrame)){
        LOG(ERROR)<<"size abnormal:("<<data.size()<<")";
        return false;
      }
      else{
        std::vector<uint8_t> multiline_laser_data(data.begin(),
            data.end() + sizeof(MultilinelaserFrame));
        data.erase(data.begin(), data.begin() + sizeof(MultilinelaserFrame));
        MultilinelaserFrame *multiline_laser_frame = (MultilinelaserFrame*)multiline_laser_data.data();
        Time now = GetNow();
        for(size_t i = 0; i<8; i++){
          Time timestamp = now - (8-i)/kMultilinelaserFrequence;                    
        }
        // LOG(ERROR)<<"yaw:(================="<<double(multiline_laser_frame->block[0].azimuth)/100<<").";
        static size_t i = 0;
        for(auto&& block:multiline_laser_frame->block){
          i++;
          Eigen::Vector3f rpy = Eigen::Vector3f::Zero();
          if(!black_board_ptr_->GetLatestRpy(rpy)){
            // LOG_EVERY_N(ERROR,32000)<<"No rpy data.("<<(rpy.x())<<","<<(rpy.y())<<","<<(rpy.z())<<")";
            continue;
          } else{
            // LOG_EVERY_N(ERROR,32000)<<"Rpy data.("<<(rpy.x())<<","<<(rpy.y())<<","<<(rpy.z())<<")";
          }

          static double last_yaw = DegToRad(double(block.azimuth)/100);
          double yaw = DegToRad(double(block.azimuth)/100);
          for(size_t j = 0;j<16;j++){
            double pitch = DegToRad(kMultilinelaserPitchOffset[j]) + sin(rpy.z()) * rpy.x() + cos(rpy.z()) * rpy.y();
            double range = double(block.channel[j].distance)*0.004;
            if(range<0.05) continue;
            uint8_t intensity = block.channel[j].reflectivity;
            // if(intensity<uint8_t(int8_t(200))) continue;
            Eigen::Vector3f position = kMultilinelaserLink+Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                                        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                                        Eigen::Vector3f(range, 0., 0.);

            // if(range>200)LOG(ERROR)<<"position:("<<position.x()<<","<<position.y()<<","<<position.z()<<")";
            geometry_msgs::Point32 pt;
            pt.x = position.x();
            pt.y = position.y();
            pt.z = position.z();
            cloud_.points.push_back(pt);
            cloud_.channels[0].values.push_back(intensity);
            if(kPublishIntensityCloud && intensity>uint8_t(int8_t(200))){
              uint32_t address = uint32_t(uint16_t(i))<<16|uint16_t(j);
              intensity_cloud_.points.push_back(pt);
              intensity_cloud_.channels[0].values.push_back(intensity);     
              address_cloud_.insert(std::pair<uint32_t,Eigen::Vector3f>(address,position));       
            }
          }

          if(last_yaw > 2*kPI-0.1&& yaw < 0.1) {      
            i = 0;  
            if(kPublishMultilinelaser) Publish();
            if(kPublishIntensityCloud) IntensityCloudPublish();
            if(kBlackBoardIntensityCloud) {
              Time now = GetNow();
              TimeAddressCloud time_address_cloud(now, address_cloud_);
              black_board_ptr_->InsertTimeAddressCloud(time_address_cloud);  
            }
              
            // address_cloud.clear();
            // if(kBlackBoardIntensityCloud) black_board_ptr_->Insert(intensity_cloud_);
          }
          last_yaw = yaw;
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

void Multilinelaser::IntensityCloudPublish(){
  intensity_cloud_.header.stamp = ros::Time::now();
  intensity_cloud_publisher_.publish(intensity_cloud_);
  intensity_cloud_.points.clear();
  intensity_cloud_.channels[0].values.clear();
};
void Multilinelaser::MultilinelaserRxThread() {
  ros::Rate rate(kMultilinelaserFrequence);
  std::vector<uint8_t> rx_data;
  LOG(ERROR) << name_ << "Multi line laser thread started!";
  while (true) {
    if (is_running_ && ReadFromIO(rx_data)) ParseData(rx_data);
    rate.sleep();
  }
}