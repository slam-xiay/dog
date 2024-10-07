#ifndef SERVICE_DOCKER__DETECTOR_HPP_
#define SERVICE_DOCKER__DETECTOR_HPP_
#include <Eigen/Core>
#include <Eigen/Dense>
#include "../blackboard.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>
#include "common/plane.hpp"


class DockerDetector {
public:
  DockerDetector(std::shared_ptr<BlackBoard> black_board_ptr_);
  ~DockerDetector();
  DockerDetector(const DockerDetector &) = delete;
  DockerDetector &operator=(const DockerDetector &) = delete;
  std::vector<std::vector<Eigen::Vector3f>> SegementCloudsByContinous(const  std::vector<Eigen::Vector3f>& cloud);
  Eigen::Vector3f GetCloudRange(const std::vector<Eigen::Vector3f>& cloud);
  Eigen::Vector3f GetCloudCenter(const std::vector<Eigen::Vector3f>& cloud);
  bool IsFitDockerRange(const Eigen::Vector3f& range);
  void CloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
  bool FitPlane(const std::vector<Eigen::Vector3f>& cloud);
  void Publish(const Eigen::Vector3f& center,const Eigen::Vector3f& normal);
 private:
    std::shared_ptr<BlackBoard> black_board_ptr_;  
    ros::Publisher docker_pose_publisher_;
    ros::Subscriber cloud_subscriber_;
    
};
#endif