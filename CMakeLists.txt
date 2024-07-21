cmake_minimum_required(VERSION 3.0.2)
project(driver)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "RELEASE")
find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs std_msgs std_srvs tf
        tf2 tf2_ros tf2_geometry_msgs geometry_msgs dynamic_reconfigure)
find_package(Boost REQUIRED)
find_package(Eigen3 QUIET)
find_package(yaml-cpp REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/src)
file(GLOB_RECURSE SRCS "src/*.cpp")
add_executable(${PROJECT_NAME}_node ${SRCS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES} 
        jpeg yaml-cpp glog gflags)