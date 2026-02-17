#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuBias.h>

#include "mvp_localization/imu_processor.hpp"
#include "mvp_localization/gps_processor.hpp"

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode();

private:
    // Sensor processors
    std::shared_ptr<IMUProcessor> imu_processor_;
    std::shared_ptr<GPSProcessor> gps_processor_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // GTSAM
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_values_;
    gtsam::ISAM2 isam_;
    gtsam::NavState prev_state_;
    gtsam::imuBias::ConstantBias prev_bias_;

    int key_;

    void initializeGTSAM();
    void optimize(const rclcpp::Time &stamp);
    void publishOdometry(const gtsam::Pose3 &pose,
                         const gtsam::Vector3 &vel,
                         const rclcpp::Time &stamp);
};
