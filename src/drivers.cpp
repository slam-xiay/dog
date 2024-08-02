#include "drivers.hpp"
Drivers::Drivers() {
  node_handle_ptr_ = std::make_shared<ros::NodeHandle>("");
  // if (node_handle_ptr_->hasParam("drivers")) {
  //   XmlRpc::XmlRpcValue drivers;
  //   node_handle_ptr_->getParam("drivers", drivers);
  //   for (int32_t i = 0; i < drivers.size(); ++i) {
  //     std::string type = std::string(drivers[i]["type"]);
  //     std::string serial_port = std::string(drivers[i]["comName"]);
  //     int frequence = 10;

  //     if (type == "ultrasonic") {
  //       std::shared_ptr<Ultrasonic> ultrasonic_ptr =
  //           std::make_shared<Ultrasonic>(node_handle_ptr_, serial_port,
  //           frequence);
  //       std::shared_ptr<std::thread> ultrasonic_thread_ptr =
  //           std::make_shared<std::thread>(&Ultrasonic::UltrasonicRxThread,
  //                                         ultrasonic_ptr);
  //       ultrasonic_ptrs_.push_back(ultrasonic_ptr);
  //       ultrasonic_thread_ptrs_.push_back(ultrasonic_thread_ptr);
  //     }
  //   }
  // }

  // for (auto&& serial_port : kUltrasonicPorts) {
  //   std::shared_ptr<Ultrasonic> ultrasonic_ptr =
  //       std::make_shared<Ultrasonic>(node_handle_ptr_, serial_port);
  //   std::shared_ptr<std::thread> ultrasonic_thread_ptr =
  //       std::make_shared<std::thread>(&Ultrasonic::UltrasonicRxThread,
  //                                     ultrasonic_ptr);
  //   ultrasonic_ptrs_.push_back(ultrasonic_ptr);
  //   ultrasonic_thread_ptrs_.push_back(ultrasonic_thread_ptr);
  // }

  // for (auto&& serial_port : kLinelaserPorts) {
  //   std::shared_ptr<Linelaser> linelaser_ptr =
  //       std::make_shared<Linelaser>(node_handle_ptr_, serial_port);
  //   std::shared_ptr<std::thread> linelaser_thread_ptr =
  //       std::make_shared<std::thread>(&Linelaser::LinelaserRxThread,
  //                                     linelaser_ptr);
  //   linelaser_ptrs_.push_back(linelaser_ptr);
  //   linelaser_thread_ptrs_.push_back(linelaser_thread_ptr);
  // }

  multilinelaser_ptr_ =
      std::make_shared<Multilinelaser>(node_handle_ptr_);

  multilinelaser_thread_ptr_ =
      std::make_shared<std::thread>(&Multilinelaser::MultilinelaserRxThread,
                                      multilinelaser_ptr_);

  imu_ptr_ =
      std::make_shared<Imu>(node_handle_ptr_,kImuPort);

  imu_thread_ptr_ =
      std::make_shared<std::thread>(&Imu::ImuRxThread,imu_ptr_);
};
Drivers::~Drivers() {
  // for (auto&& ultrasonic_ptr : ultrasonic_ptrs_)
  //   ultrasonic_ptr->SetRunning(false);
  // for (auto&& ultrasonic_thread_ptr : ultrasonic_thread_ptrs_) {
  //   if ((ultrasonic_thread_ptr != nullptr) &&
  //       ultrasonic_thread_ptr->joinable()) {
  //     ultrasonic_thread_ptr->join();
  //     ultrasonic_thread_ptr.reset();
  //   }
  // }
};