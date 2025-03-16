#ifndef EKF_SLAM_HPP
#define EKF_SLAM_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>


struct landmark {
    Eigen::Vector2d pos;
    int id;
};

class EKF_SLAM : public rclcpp::Node {
public:
    EKF_SLAM();

private:
    Eigen::VectorXd state_;         // [x, y, theta, m1x, m1y, ...]
    Eigen::MatrixXd cov_;
    std::vector<landmark> landmarks_;

    double last_time_;
    // Noise models
    Eigen::MatrixXd Q;              // process noise cov
    Eigen::MatrixXd R_lidar;        // sensor noise cov (measurement)
    // Eigen::MatrixXd R_cam;

    rclcpp::Subscribtion<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscribtion<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cam_sub_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg);
    // void camCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cam_msg);

    void predict(const Eigen::Vector2d& u, double dt);
    void update(const std::vector<Eigen::Vector2d>& measurements);
}

#endif