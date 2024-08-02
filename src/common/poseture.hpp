#ifndef SLAM_COMMON_POSTURE_HPP__
#define SLAM_COMMON_POSTURE_HPP__
#include <math.h>

struct Posture {
  float x;
  float y;
  float yaw;
  Posture(float x_, float y_, float yaw_) : x(x_), y(y_), yaw(yaw_) {}
        Posture() {
    this->x = 0.;
    this->y = 0.;
    this->yaw = 0.;
  }
  Posture(float pose[3]) {
    this->x = pose[0];
    this->y = pose[1];
    this->yaw = pose[2];
  }
  Posture translation() { return Posture(x, y, 0.); }
  Posture rotation() { return Posture(0., 0., yaw); }
  Posture inverse() const {
    return Posture(y * std::sin(-yaw) - x * std::cos(-yaw), -x * std::sin(-yaw) - y * std::cos(-yaw), -yaw);
  }
  float norm() { return std::sqrt(std::pow(x, 2.) + std::pow(y, 2.)); }
};
#endif