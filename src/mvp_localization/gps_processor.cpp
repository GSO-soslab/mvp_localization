#include "mvp_localization/gps_processor.hpp"
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/noiseModel.h>

GPSProcessor::GPSProcessor()
    : origin_initialized_(false)
{
    // Nothing else to initialize
}

void GPSProcessor::processGPS(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg,
    gtsam::NonlinearFactorGraph& graph,
    int key)
{
    // Skip if GPS has no fix
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
        return;

    // Initialize origin if not done
    if (!origin_initialized_)
    {
        geo_converter_.Reset(msg->latitude, msg->longitude, msg->altitude);
        origin_initialized_ = true;
        RCLCPP_INFO(rclcpp::get_logger("GPSProcessor"), "GPS origin initialized");
    }

    double x, y, z;
    geo_converter_.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);

    // GPS noise model
    auto gps_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1.0, 1.0, 2.0));

    // Add GPS factor to graph
    graph.add(gtsam::GPSFactor(X(key), gtsam::Point3(x, y, z), gps_noise));
}
