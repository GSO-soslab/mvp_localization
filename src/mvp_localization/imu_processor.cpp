#include "mvp_localization/imu_processor.hpp"
#include <gtsam/navigation/ImuFactor.h>

IMUProcessor::IMUProcessor()
{
    // Preintegration parameters
    auto params = gtsam::PreintegrationParams::MakeSharedU(9.81); // gravity
    params->accelerometerCovariance = gtsam::I_3x3 * 0.1;
    params->gyroscopeCovariance = gtsam::I_3x3 * 0.1;
    params->integrationCovariance = gtsam::I_3x3 * 1e-8;

    imu_preintegrator_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(
        params, gtsam::imuBias::ConstantBias());
}

void IMUProcessor::processIMU(
    const sensor_msgs::msg::Imu::SharedPtr msg,
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& initial_values,
    gtsam::NavState& prev_state,
    gtsam::imuBias::ConstantBias& prev_bias,
    int key)
{
    if (!initialized_)
    {
        last_time_ = msg->header.stamp;
        initialized_ = true;
        return;
    }

    double dt = (msg->header.stamp - last_time_).seconds();
    last_time_ = msg->header.stamp;

    // Acceleration and angular velocity vectors
    gtsam::Vector3 accel(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);

    gtsam::Vector3 gyro(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z);

    // Integrate measurement
    imu_preintegrator_->integrateMeasurement(accel, gyro, dt);

    // Add IMU factor to graph
    graph.add(gtsam::ImuFactor(
        X(key - 1), V(key - 1),
        X(key), V(key),
        B(key - 1),
        *imu_preintegrator_));

    // Predict next state
    gtsam::NavState predicted = imu_preintegrator_->predict(prev_state, prev_bias);

    // Insert predicted values into initial_values
    initial_values.insert(X(key), predicted.pose());
    initial_values.insert(V(key), predicted.velocity());
    initial_values.insert(B(key), prev_bias);

    // Reset preintegration
    imu_preintegrator_->resetIntegration();
}
