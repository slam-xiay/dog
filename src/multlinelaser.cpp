#include "multilinelaser.hpp"
Multilinelaser::Multilinelaser(std::shared_ptr<ros::NodeHandle> node_handle_ptr):node_handle_ptr_(node_handle_ptr){    
    address_.sin_family = AF_INET;
    address_.sin_port = htons(kLidarSourcePort);
    address_.sin_addr.s_addr = INADDR_ANY;
    fd_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (bind(fd_, reinterpret_cast<sockaddr *>(&address_),
            sizeof(sockaddr)) == -1) {
      LOG(ERROR)<<"Can not bind fd.";
      return;
    }
    if (fcntl(fd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        LOG(ERROR)<<"Non block.";
      return;
    }
};
Multilinelaser::~Multilinelaser(){};

void Multilinelaser::MultilinelaserRxThread() {
  ros::Rate rate(kLidarFrequence);
  std::vector<uint8_t> rx_data;
  LOG(ERROR) << name_ << " thread started!";
  struct pollfd fds[1];
  fds[0].fd = fd_;
  fds[0].events = POLLIN;
  sockaddr_in senderAddress;
  socklen_t senderAddressLen = sizeof(senderAddress);
    while (true) {
        int retval = poll(fds, 1, 1000);
        if (retval < 0&&errno != EINTR) 
        LOG(ERROR)<< "Poll error:("<< strerror(errno)<<").";
        if(retval>0){
            if((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
                (fds[0].revents & POLLNVAL)) LOG(ERROR)<<"poll() reports Pandar error";
            else{
                senderAddressLen = sizeof(senderAddress);
                ssize_t nbytes;
                if (fds[0].revents & POLLIN) {
                    uint8_t data[1500];
                    nbytes = recvfrom(fds[0].fd, &data[0], 1500, 0,
                    reinterpret_cast<sockaddr *>(&senderAddress),&senderAddressLen);
                    LOG(ERROR)<<"receive nbytes:("<<nbytes<<")";
                }
            }
        // if (is_running_ && ReadFromIO(rx_data)) ParseData(rx_data);
            rate.sleep();
        }
    }
}