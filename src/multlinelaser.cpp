#include "multilinelaser.hpp"// add tag 1
Multilinelaser::Multilinelaser(std::shared_ptr<ros::NodeHandle> node_handle_ptr):node_handle_ptr_(node_handle_ptr){  
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
      if (data.size() < sizeof(MultilinelaserFrame))
        return false;
      else{
        std::vector<uint8_t> multiline_laser_data(data.begin(),
            data.end() + sizeof(MultilinelaserFrame));
        data.erase(data.begin(), data.begin() + sizeof(MultilinelaserFrame));
        MultilinelaserFrame *multiline_laser_frame = (MultilinelaserFrame*)multiline_laser_data.data();
        LOG(ERROR)<<"angle:("<<double(multiline_laser_frame->block->azimuth)/100<<")";
      }
  }
  return true;
}
void Multilinelaser::MultilinelaserRxThread() {
  ros::Rate rate(kLidarFrequence);
  std::vector<uint8_t> rx_data;
  LOG(ERROR) << name_ << "thread started!";
  while (true) {
    if (is_running_ && ReadFromIO(rx_data)) ParseData(rx_data);
    rate.sleep();
  }
  // std::vector<uint8_t> rx_data;
  // LOG(ERROR) << name_ << "thread started!";
  // while(true){
  //   std::vector<uint8_t> rx_data;
  //   if (fd_ == -1) continue;
  //   uint8_t* rx_buf = new uint8_t[kMaxSerialBuf];
  //   size_t rx_len = 0;
  //   static timespec timeout = {0, (long)(kPerReadTimeoutns)};  // 0.1ms
  //   fd_set read_fds;
  //   FD_ZERO(&read_fds);
  //   FD_SET(fd_, &read_fds);
  //   int r = pselect(fd_ + 1, &read_fds, NULL, NULL, &timeout, NULL);
  //   if (r <= 0) continue;
  //   if (FD_ISSET(fd_, &read_fds)) rx_len = read(fd_, rx_buf, kMaxSerialBuf);
  //   // LOG(ERROR)<<"size:("<<rx_len<<")";
  //   // printf("\n====\n");
  //   // for (int i = 0; i < rx_len; i++) {
  //   //   printf("%2x ",*(rx_buf + i));
  //   //   // data.push_back(*(rx_buf + i));      
  //   // }
  //   // printf("\n****\n");
  //   rate.sleep();
  // }
  // struct pollfd fds[1];
  // fds[0].fd = fd_;
  // fds[0].events = POLLIN;
  // sockaddr_in senderAddress;
  // socklen_t senderAddressLen = sizeof(senderAddress);

  

  // int connect_res = -1;
  // while (true) {
    // while(true){
    //   int connect_res = connect(fd_, (struct sockaddr*)&address_, sizeof(address_));
    //   LOG(ERROR)<<"Connect:("<<connect_res<<").";
    //   if(connect_res>=0) break;
    //   sleep(0.1);
    // }
    // int nrecvSize = 0;
    // char msg[1024];
    // int read_res = read(fd_, msg, 1000);
    // LOG(ERROR)<<"Read response:("<<read_res<<").";
// if((nrecvSize = read(fd_, msg, 1000)) < 0)	//接受数据
// 		{
// 			printf("read Error: %s (errno: %d)\n", strerror(errno), errno);
// 		}
// 		else if(nrecvSize == 0)
// 		{
// 			printf("Service Close!\n");
// 		}
// 		else
// 		{
// 			printf("Server return: %s\n", msg);
// 		}


    // int retval = poll(fds, 1, 1000);
    // LOG(ERROR)<<"retval:("<<retval<<")";
    //   if (retval < 0&&errno != EINTR) 
    //   LOG(ERROR)<< "Poll error:("<< strerror(errno)<<").";
    //   if(retval>0){
    //       if((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
    //           (fds[0].revents & POLLNVAL)) LOG(ERROR)<<"poll() reports Pandar error";
    //       else{
    //           senderAddressLen = sizeof(senderAddress);
    //           ssize_t nbytes;
    //           if (fds[0].revents & POLLIN) {
    //               uint8_t data[1500];
    //               nbytes = recvfrom(fds[0].fd, &data[0], 1500, 0,
    //               reinterpret_cast<sockaddr *>(&senderAddress),&senderAddressLen);
    //               LOG(ERROR)<<"receive nbytes:("<<nbytes<<")";
    //           }
    //       }
    //   // if (is_running_ && ReadFromIO(rx_data)) ParseData(rx_data);
    //       rate.sleep();
    //   }
  // }
}