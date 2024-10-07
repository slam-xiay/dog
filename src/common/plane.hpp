#ifndef COMMON_PLANE_HPP_
#define COMMON_PLANE_HPP_
#include <Eigen/Dense>
struct Plane {
  Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  Eigen::Vector4f coeffs = Eigen::Vector4f::Zero();
  std::vector<Eigen::Vector3f> points;
  Plane(){};
};
// bool FitPlane(std::vector<Eigen::Vector3f>& points, Plane& plane){
//     return true;
// };
// bool IsFitMse(Plane& plane){
//     return true;
// };
#endif
