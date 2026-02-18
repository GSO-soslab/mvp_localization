#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "geographic_msgs/msg/geo_point.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/MagneticModel.hpp>

// GTSAM includes
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Velocity (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias (ax,ay,az,gx,gy,gz)

class GtsamImuNode : public rclcpp::Node {
public:
    GtsamImuNode() : Node("gtsam_imu_node"), initialized_(false), count_(0) {
        
        // 1. Define IMU Parameters (Sensor Specs)
        auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(9.81);
        p->accelerometerCovariance = I_3x3 * pow(0.0003924, 2);
        p->gyroscopeCovariance = I_3x3 * pow(0.0002056, 2);
        p->integrationCovariance = I_3x3 * 1e-8;
        
        // Initialize Preintegration object with zero bias
        imuBias::ConstantBias prior_bias; 
        pim_ = std::make_shared<PreintegratedImuMeasurements>(p, prior_bias);

        // 2. ROS Setup
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&GtsamImuNode::imuCallback, this, std::placeholders::_1));
        
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps/fix", 10, std::bind(&GtsamImuNode::gpsCallback, this, std::placeholders::_1));
        
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 10);
        
        // 3. ISAM2 Setup
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam2_ = std::make_unique<ISAM2>(parameters);

        RCLCPP_INFO(this->get_logger(), "GTSAM Node started. Waiting for first GPS/Odom to initialize...");
    }

private:
    void initializeSystem(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {


        m_datum.latitude = msg->latitude;

        m_datum.longitude = msg->longitude;


        // Rot3 prior_rotation = Rot3::Quaternion(
        //     msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        //     msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        
        Rot3 prior_rotation = Rot3::Quaternion(
            1,0,
           0,0);
            // Point3 prior_point(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Point3 prior_point(0, 0, 0);

        Pose3 prior_pose(prior_rotation, prior_point);
        Vector3 prior_velocity(0, 0, 0);
        imuBias::ConstantBias prior_bias;

        // Add initial estimates
        initial_values_.insert(X(0), prior_pose);
        initial_values_.insert(V(0), prior_velocity);
        initial_values_.insert(B(0), prior_bias);

        // Add Priors
        auto pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.3, 0.3, 0.3).finished());
        auto vel_noise = noiseModel::Isotropic::Sigma(3, 0.1);
        auto bias_noise = noiseModel::Isotropic::Sigma(6, 1e-3);

        graph_.addPrior(X(0), prior_pose, pose_noise);
        graph_.addPrior(V(0), prior_velocity, vel_noise);
        graph_.addPrior(B(0), prior_bias, bias_noise);

        isam2_->update(graph_, initial_values_);
        
        // Reset for next cycle
        graph_.resize(0);
        initial_values_.clear();

        prev_state_ = NavState(prior_pose, prior_velocity);
        prev_bias_ = prior_bias;
        initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "System Initialized.");
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        if (!initialized_) {
            last_imu_time_ = current_time;
            return;
        }

        double dt = current_time - last_imu_time_;
        if (dt <= 0) return;

        last_imu_time_ = current_time;

        // Integrate
        Vector3 acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        Vector3 gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        pim_->integrateMeasurement(acc, gyro, dt);
    }

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (!initialized_) {

            initializeSystem(msg);
            return;
        }

        //convert gps into x and y using datum
        geographic_msgs::msg::GeoPoint ll_point;
        geometry_msgs::msg::Point::SharedPtr map_point = std::make_shared<geometry_msgs::msg::Point>();
        ll_point.latitude = msg->latitude;
        ll_point.longitude = msg->longitude;
        ll_point.altitude = msg->altitude;

        f_ll2dis(ll_point, map_point);
        
        count_++;

        // 1. Add IMU Factor
        ImuFactor imu_factor(X(count_ - 1), V(count_ - 1), X(count_), V(count_), B(count_ - 1), *pim_);
        graph_.add(imu_factor);

        // 2. Add GPS Factor (Position only)
        Point3 gps_pos(map_point->x, map_point->y, map_point->z);
        auto gps_noise = noiseModel::Isotropic::Sigma(3, 0.1);
        graph_.add(GPSFactor(X(count_), gps_pos, gps_noise));

        // 3. Add Bias Random Walk (BetweenFactor)
        auto bias_noise = noiseModel::Isotropic::Sigma(6, 1e-3);
        graph_.add(BetweenFactor<imuBias::ConstantBias>(B(count_ - 1), B(count_), imuBias::ConstantBias(), bias_noise));

        // 4. Predict and optimize
        NavState prop_state = pim_->predict(prev_state_, prev_bias_);
        initial_values_.insert(X(count_), prop_state.pose());
        initial_values_.insert(V(count_), prop_state.v());
        initial_values_.insert(B(count_), prev_bias_);

        isam2_->update(graph_, initial_values_);
        Values result = isam2_->calculateEstimate();

        // 5. Update for next iteration
        prev_state_ = NavState(result.at<Pose3>(X(count_)), result.at<Vector3>(V(count_)));
        prev_bias_ = result.at<imuBias::ConstantBias>(B(count_));

        pim_->resetIntegrationAndSetBias(prev_bias_);
        graph_.resize(0);
        initial_values_.clear();

        publishOdometry(msg->header.stamp);
    }

    void publishOdometry(const rclcpp::Time& stamp) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        Pose3 pose = prev_state_.pose();
        odom.pose.pose.position.x = pose.x();
        odom.pose.pose.position.y = pose.y();
        odom.pose.pose.position.z = pose.z();

        Quaternion q = pose.rotation().toQuaternion();
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();

        Vector3 vel = prev_state_.v();
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();

        odom_pub_->publish(odom);
    }

    void f_ll2dis(geographic_msgs::msg::GeoPoint ll_point, geometry_msgs::msg::Point::SharedPtr map_point)
    {
        // double north = m_earthR*(ll_point.latitude - m_datum.latitude)/180.0*M_PI;
        // double east = m_earthR*cos(m_datum.latitude/180.0*M_PI) * (ll_point.longitude - m_datum.longitude)/180.0*M_PI;
        
        //in world frame.
        //Geographic lib implementation
        double distance, azimuth1, azimuth2;
        const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
        geod.Inverse(m_datum.latitude, m_datum.longitude, 
                    ll_point.latitude, ll_point.longitude, 
                    distance, azimuth1, azimuth2);
        double north = distance *cos(azimuth1/180.0*M_PI);
        double east = distance * sin(azimuth1/180.0*M_PI);

        map_point->x = east;
        map_point->y = north;
        map_point->z = ll_point.altitude;
    }

    void f_dis2ll(geometry_msgs::msg::Point map_point, geographic_msgs::msg::GeoPoint::SharedPtr ll_point)
    {
        //from world frame
        // double lat = m_datum.latitude + map_point.y/m_earthR * 180.0/M_PI;
        // double lon = m_datum.longitude + map_point.x/(m_earthR*cos(m_datum.latitude/180.0*M_PI)) * 180.0/M_PI;
        const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
        double lat, lon;
        // Calculate latitude and longitude of target point
        geod.Direct(m_datum.latitude, m_datum.longitude, 0.0, map_point.y, lat, lon); // Move north
        geod.Direct(lat, lon, 90.0, map_point.x, lat, lon); // Move east
        
        ll_point->latitude = lat;
        ll_point->longitude = lon;
        ll_point->altitude = map_point.z;

    }

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

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    geographic_msgs::msg::GeoPoint m_datum;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GtsamImuNode>());
    rclcpp::shutdown();
    return 0;
}