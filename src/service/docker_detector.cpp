#include "docker_detector.hpp"

DockerDetector::DockerDetector(std::shared_ptr<BlackBoard> black_board_ptr):black_board_ptr_(black_board_ptr) {
    docker_pose_publisher_ = black_board_ptr_->GetNodeHandlePtr()->advertise<geometry_msgs::PoseStamped>("docker_pose", 1);
    cloud_subscriber_ = black_board_ptr_->GetNodeHandlePtr()->subscribe<sensor_msgs::PointCloud>
        ("/intensity_cloud", 10, &DockerDetector::CloudCallback, this);
}
DockerDetector::~DockerDetector() {    
}

std::vector<std::vector<Eigen::Vector3f>> DockerDetector::SegementCloudsByContinous(const  std::vector<Eigen::Vector3f>& cloud){
    std::vector<std::vector<Eigen::Vector3f>> continous_clouds;
    for(auto&& point: cloud){
        bool is_inserted = false;
        for(auto&& continous_cloud:continous_clouds){
            for(auto&& continous_point:continous_cloud){
                if((point-continous_point).norm()<kMinPlanePointDistance){
                    continous_cloud.push_back(point);
                    is_inserted = true;
                    break;
                }
            }
        }
        if(!is_inserted){
            std::vector<Eigen::Vector3f> continous_cloud;
            continous_cloud.push_back(point);
            continous_clouds.push_back(continous_cloud);
        }
    }
    return continous_clouds;
}

Eigen::Vector3f DockerDetector::GetCloudRange(const std::vector<Eigen::Vector3f>& cloud){
    Eigen::Vector3f min,max;
    bool is_initial = false;
    if(cloud.empty())
        return Eigen::Vector3f::Zero();
    else{
        for(auto&& point:cloud){
            if(!is_initial) {
                min = point;
                max = point;
                is_initial = true;
            }else{
                min = min.cwiseMin(point);
                max = max.cwiseMax(point);
            }        
        }
        return max-min;
    }
}

Eigen::Vector3f DockerDetector::GetCloudCenter(const std::vector<Eigen::Vector3f>& cloud){
    Eigen::Vector3f center(0.f,0.f,0.f);
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    for(auto&& point:cloud)
        center += point;  
    center /= cloud.size();
    return center;
}

bool DockerDetector::IsFitDockerRange(const Eigen::Vector3f& range){
    float normal = range.norm();
    return normal<kMaxDockerRange&&normal>kMinDockerRange;    
}
void DockerDetector::CloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    std::vector<Eigen::Vector3f> cloud;
    for(auto&& point: msg->points)
        cloud.push_back(Eigen::Vector3f(point.x, point.y, point.z));    
    std::vector<std::vector<Eigen::Vector3f>> continous_clouds = SegementCloudsByContinous(cloud);
    if(continous_clouds.empty()){
        LOG_EVERY_N(ERROR,1)<<"Continous intensity clouds is empty,and no docker is detected.";        
    }else{
        for(auto&& continous_cloud:continous_clouds){
            LOG(ERROR)<<"Cloud size:"<<continous_cloud.size();
            LOG(ERROR)<<"IsFitDockerRange:("<<IsFitDockerRange(GetCloudRange(continous_cloud))<<")";
            if(continous_cloud.size()>kMinPlaneSize&&IsFitDockerRange(GetCloudRange(continous_cloud))&&FitPlane(continous_cloud)){
                LOG(ERROR)<<"Publish docker pose.";
            }
        }
    }    
}

//Ax+By+Cz+D = 0;
bool DockerDetector::FitPlane(const std::vector<Eigen::Vector3f>& cloud){
    Eigen::Vector3f center = GetCloudCenter(cloud);
    Eigen::Matrix<float, kMinPlaneSize, 3> A;
    Eigen::Matrix<float, kMinPlaneSize, 1> B;
    float distance = std::pow(center.x(), 2.) + std::pow(center.y(), 2.) + std::pow(center.z(), 2.);
    size_t interval = cloud.size() / kMinPlaneSize;
    for (size_t i = 0; i < kMinPlaneSize; i++) {
        A(i, 0) = cloud[i * interval].x()-center.x();
        A(i, 1) = cloud[i * interval].y()-center.y();
        A(i, 2) = cloud[i * interval].z()-center.z();
        B(i ) = -distance;
    }
    Eigen::Matrix<float, 3, 1> X = A.colPivHouseholderQr().solve(B);
    Eigen::Vector3f normal(X[0]/distance,X[1]/distance,X[2]/distance);
    Publish(center,normal);
    return true;
}

void DockerDetector::Publish(const Eigen::Vector3f& center,const Eigen::Vector3f& normal){
    geometry_msgs::PoseStamped docker_pose;
    docker_pose.header.stamp = ros::Time::now();
    docker_pose.header.frame_id = "base_link";
    docker_pose.pose.position.x = center.x();
    docker_pose.pose.position.y = center.y();
    docker_pose.pose.position.z = center.z();
    docker_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(normal.x(),normal.y(),normal.z());
    docker_pose_publisher_.publish(docker_pose);
}