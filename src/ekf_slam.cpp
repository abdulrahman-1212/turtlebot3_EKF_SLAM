#include "ekf_slam.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>


EKF_SLAM::EKF_SLAM() : Node("ekf_slam_node"), last_time_(-1.0) {
    state_ = Eigen::VectorXd::Zero(3);
    cov_ = Eigen::MatrixXd::Identity(3, 3) * 0.1;

    Q = Eigen::MatrixXd::Identity(3, 3) * 0.01;                   // (x, y, theta)
    R_lidar = Eigen::MatrixXd::Identity(2, 2) * 0.05;            // (range, bearing)
    // R_cam = Eigen::MatrixXd::Identity(2, 2) * 0.1;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry> (
        "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odomCallback(msg); });
    lidar_sub_ create_subscription<sensor_msgs::msg::LaserScan> (
        "/scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { lidarCallback(msg); });
    // cam_sub_ = create_subscription<sensor_msgs::msg::PoitnCloud2> (
    //     "/depth/points", 10, std::bind(&EKF_SLAM::camCallback, this, std::placeholder::_1));

    RCLCPP_INFO(this->get_logger(), "EKF SLAM with LiDAR and Depth Camera Initialized!");
};

void EKF_SLAM::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
   double current_time = odom_msg->header.stamp.sec + odom_msg->header.stamp.nansec * 1e-9;
   if (last_time_ < 0) {
    last_time_ = current_time;
    return;
   }

   double dt = current_time - last_time_;
   last_time_ = current_time;

   Eigen::Vector2d u(
    odom_msg->twist.twist.linear.x,
    odom_msg->twist.twist.angular.z
   );

    predict(u, dt);
}

void EKF_SLAM::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg) {
    std::vector<Eigen::Vector2d> measurements;
    update(measurements);
}


void EKF_SLAM::predict(const Eigen::Vector2d& u, double dt) {
    double v = u[0];
    double omega = u[1];
    double theta = state_[2];
    // Update the state 
    state_[0] += v * dt * cos(theta);
    state_[1] += v * dt * sin(theta);
    state_[2] += omega * dt;

    // Computing the Jacobian of motion model
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_.size());
    F(0, 2) = -v * dt * sin(theta);             // dx/dtheta
    F(1, 2) = v * dt * cos(theta);              // dy/dtheta
    // Jacobian of Control
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(3, 2);
    G(0, 0) = dt * cos(theta);
    G(1, 0) = dt * sin(theta);
    G(2, 1) = dt;

    cov_.block(0, 0, 3, 3) = F * cov_.block(0, 0, 3, 3) * F.transpose() + G * Q * G.transpose();        // Q is process noise cov
}

void EKF_SLAM::update(const std::vector<Eigen::Vector2d>& measurements) {

    for  (size_t i = 0; i < measurements.size(); i++) {
        landmark_idx = i < landmarks_.size() ? i : landmarks_.size();
        if (i >= landmraks_.size()) {
            double x = state_[0] + measurements[i][0] * cos(state_[2] + measurements[i][1]);
            double y = state_[1] + measurements[i][0] * sin(state_[2] + measurements[i][1]);
            state_.conservativeResize(state_.size() + 2);
            state_(state_.size() - 2) = x;
            state_(state_.size() - 1) = y;
            cov_.conservativeResize(state_.size(), state_.size());
            cov_.bottomRightCorner(2, 2) = Eigen::Matrix2d::Identity() * 1.0;
            landmarks_.push_back({Eigen::Vector2d(x, y), landmark_idx});

        }

        Eigen::Vector2d z_pred;
        double dx = landmarks_[landmark_idx].pos[0] - state_[0];
        double dy = landmarks_[landmark_idx].pos[1] - state_[1];
        double r = sqrt(dx * dx + dy * dy);
        z_pred[0] = r;                              // range
        z_pred[1] = atan2(dy, dx) - state_[2];      // bearing
    
        Eigen::MatrixXd::Zero H(2, state_.size);
        H(0, 0) = -dx / r;                      // dr/dx
        H(0, 1) = -dy/r;                        // dr/dy
        H(1, 0) = dy / (r * r);                 // dphi/dx        
        H(1, 1) = -dx / (r * r);                // dphi/dy
        H(1, 2) = -1.0;                         // dphi/dtheta
        H(0, 3 + 2 * landmark_idx) = dx / r;    // dr/dmx
        H(0, 4 + 2 * landmark_idx) = dy / r;    // dr/dmy
        H(1, 3 + 2 * landmark_idx) = -dy / (r * r);     // dphi/dmx
        H(1, 4 + 2 * landmark_idx) = dx / (r * r);      // dphi/dmy
    
        Eigen::Vector2d y = measurements[i] - z_pred;
        Eigen::MatrixXd S = H * cov_ * H.transpose() + R_lidar_;
        Eigen::MatrixXd K = cov_ * H.transpose() * S.inverse();

        state_ += K * y;
        cov_ = (Eigen::MatrixXd::Identity(state_.size(), state.size()) - K * H) * cov_;

    }
}


