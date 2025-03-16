#ifndef LANDMARK_DETECTOR_HPP
#define LANDMARK_DETECTOR_HPP

#include <sensor_msgs/msg/laser_scan.hpp>
#include <Eigen/Dense>
#include <vector>

class LandmarkDetector {
public:
    std::vector<Eigen::Vector2d> detect(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

private:
    double cluster_threshold_ = 0.1; 
}

#endif