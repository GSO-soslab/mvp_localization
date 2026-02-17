#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/PreintegratedImuMeasurements.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

class IMUProcessor
{
public:
    IMUProcessor();

    /**
     * Process a single IMU message:
     * Integrates measurement and adds IMU factor to the graph.
     */
    void processIMU(
        const sensor_msgs::msg::Imu::SharedPtr msg,
        gtsam::NonlinearFactorGraph& graph,
        gtsam::Values& initial_values,
        gtsam::NavState& prev_state,
        gtsam::imuBias::ConstantBias& prev_bias,
        int key);

private:
    std::shared_ptr<gtsam::PreintegratedImuMeasurements> imu_preintegrator_;
    rclcpp::Time last_time_;
    bool initialized_ = false;
};
