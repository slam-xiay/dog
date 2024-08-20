#include "imu.hpp"
Imu::Imu(std::shared_ptr<BlackBoard> black_board_ptr, const std::string &serial_port)
    : black_board_ptr_(black_board_ptr),
      Serial(serial_port) {
  name_ = serial_port.substr(5);
  publisher_ = black_board_ptr_->GetNodeHandlePtr()->advertise<sensor_msgs::Imu>(name_, 1);
  switch_ = black_board_ptr_->GetNodeHandlePtr()->advertiseService(
      name_, &Imu::SwitchCallBack, this);
  // Close();
  // Open(kBandrateCode921600);
  is_running_=true;
}

Imu::~Imu(){
  Close();
}
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

  while (data.size() > sizeof(ImuFrame)) {
    size_t drop = 0;
    while(data.size() > sizeof(ImuFrame)){
      if(data[0] == kImuHeader1 && data[1]==kImuAddress1 && data[2]==kImuAddress2){
        break;
      }else{
        drop++;
        data.erase(data.begin());
      }
    }
    if(drop) LOG(ERROR) << "Drop size:(" << drop << "),data size:("<<data.size()<<").";
    if(data.size()<sizeof(ImuFrame)) return true;
    std::vector<uint8_t> imu_data(data.begin(),data.begin() + sizeof(ImuFrame));
    data.erase(data.begin(), data.begin() + sizeof(ImuFrame));
    ImuFrame *imu_frame = (ImuFrame*)imu_data.data();
    size_t length = size_t(uint16_t(imu_data[6]) << 8 | uint16_t(imu_data[5]));
    if(length!=kImuContentSize){
      LOG(ERROR)<<"Imu frame length error:("<<length<<"),("<<size_t(imu_frame->length)<<").";
      continue;
    } 
    if(imu_frame->end[0]!=kImuEnd1||imu_frame->end[1]!=kImuEnd2){
      LOG(ERROR)<< std::setfill('0') << std::setw(2) << std::hex <<"Imu frame tail error:(Ox"
        <<int(imu_frame->end[0])<<"),(Ox"<<int(imu_frame->end[1])<<")";
      continue;
    }
    if(!crc_check(imu_data)){
      LOG(ERROR)<<"Imu frame crc error.";
      continue;
    }
    if(kPublishImu) Publish(imu_frame);
    if(kBlackBoardImu) CachePry(imu_frame);
  }
  return true;
}

void Imu::CachePry(const ImuFrame *imu_frame){
  times_.push_back(ros::Time::now());
  black_board_ptr_->InsertTimePry(TimePry{GetNow(),
      Eigen::Vector3f(imu_frame->euler[0],imu_frame->euler[1],imu_frame->euler[2])});
  while (times_.size() > 100) {
    times_.erase(times_.begin());
    LOG_EVERY_N(ERROR, 100)
        << name_ << " cache pry frequence in last 100 message:("
        << double(times_.size() - 1) /
               ((times_.back() - times_.front()).toSec())
        << ")hz.";
  }
}

void Imu::Publish(const ImuFrame *imu_frame) {
  sensor_msgs::Imu msg;
  msg.header.frame_id = name_;
  msg.header.stamp = ros::Time::now();
  msg.linear_acceleration.x = imu_frame->corrected_acc[0];
  msg.linear_acceleration.y = imu_frame->corrected_acc[1];
  msg.linear_acceleration.z = imu_frame->corrected_acc[2];
  msg.angular_velocity.x = imu_frame->corrected_gyro0[0];
  msg.angular_velocity.y = imu_frame->corrected_gyro0[1];
  msg.angular_velocity.z = imu_frame->corrected_gyro0[2];
  msg.orientation.x = imu_frame->quaternion[0];
  msg.orientation.y = imu_frame->quaternion[1];
  msg.orientation.z = imu_frame->quaternion[2];
  msg.orientation.w = imu_frame->quaternion[3];
  publisher_.publish(msg);  
  times_.push_back(msg.header.stamp);
  while (times_.size() > 100) {
    times_.erase(times_.begin());
    LOG_EVERY_N(ERROR, 100)
        << name_ << " publish frequence in last 100 message:("
        << double(times_.size() - 1) /
               ((times_.back() - times_.front()).toSec())
        << ")hz.";
  }
}

void Imu::SetRunning(const bool &running) {
  if (running) {
    // Close();
    Open(kBandrateCode921600);
  } else {
    Close();
  }
  is_running_ = running;
}

bool Imu::SwitchCallBack(std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res) {
  SetRunning(req.data);
  return true;
};
