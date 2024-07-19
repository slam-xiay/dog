#ifndef CAMERA_HPP
#define CAMERA_HPP
#include <glog/logging.h>
#include <signal.h>
#include <thread>
class Camera {
  public:
  Camera();
  ~Camera();
  Camera(const Camera &) = delete;
  Camera &operator=(const Camera &) = delete;

  private:
            // std::unique_ptr camera_thread_;
};

#endif  // !CAMERA_HPP
