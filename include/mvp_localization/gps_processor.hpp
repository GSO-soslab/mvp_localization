#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <GeographicLib/LocalCartesian.hpp>

using gtsam::symbol_shorthand::X;

class GPSProcessor
{
public:
    GPSProcessor();

    /**
     * Process a single GPS message and add a factor to the graph
     */
    void processGPS(
        const sensor_msgs::msg::NavSatFix::SharedPtr msg,
        gtsam::NonlinearFactorGraph& graph,
        int key);

private:
    GeographicLib::LocalCartesian geo_converter_;
    bool origin_initialized_;
};
