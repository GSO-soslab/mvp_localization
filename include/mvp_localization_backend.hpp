#ifndef MVP_LOCALIZATION_BACKEND_HPP_
#define MVP_LOCALIZATION_BACKEND_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

// GTSAM includes
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

// GeographicLib
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/MagneticModel.hpp>

#include <mutex>

using namespace gtsam;
using symbol_shorthand::B; // Bias
using symbol_shorthand::V; // Velocity
using symbol_shorthand::X; // Pose

class MvpLocalizationBackend : public rclcpp::Node {
public:
    MvpLocalizationBackend();
    virtual ~MvpLocalizationBackend() = default;

private:
    // ROS callbacks
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void dvlCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void depthCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // ROS sub and pub
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr depth_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    void publishOdomCallback();

    // Helper Methods
    void initializeSystem(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    // void publishOdometry(const rclcpp::Time& stamp, const NavState& state);
    void f_ll2dis(geographic_msgs::msg::GeoPoint ll_point, geometry_msgs::msg::Point::SharedPtr map_point);
    void f_dis2ll(geometry_msgs::msg::Point map_point, geographic_msgs::msg::GeoPoint::SharedPtr ll_point);

    // Members
    bool initialized_;
    uint64_t count_;
    double last_imu_time_;
    
    NavState prev_state_;
    imuBias::ConstantBias prev_bias_;

    std::shared_ptr<PreintegratedImuMeasurements> pim_;
    NonlinearFactorGraph graph_;
    Values initial_values_;
    std::unique_ptr<ISAM2> isam2_;

    
    rclcpp::TimerBase::SharedPtr timer_;

    geographic_msgs::msg::GeoPoint m_datum;
    std::mutex data_mutex_;
};

#endif // GTSAM_IMU_NODE_HPP_