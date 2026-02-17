#include "mvp_localization/mvp_localization.hpp"

LocalizationNode::LocalizationNode()
    : Node("localization_node"), key_(0)
{
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 50);

    // Initialize processors
    imu_processor_ = std::make_shared<IMUProcessor>();
    gps_processor_ = std::make_shared<GPSProcessor>();

    initializeGTSAM();

    // IMU subscriber
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 200,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg){
            // Pass to IMU processor
            imu_processor_->processIMU(msg, graph_, initial_values_, prev_state_, prev_bias_, key_);
            // Optimize after IMU factor
            optimize(msg->header.stamp);
            key_++;
        });

    // GPS subscriber
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 10,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg){
            gps_processor_->processGPS(msg, graph_, key_);
        });
}

void LocalizationNode::initializeGTSAM()
{
    prev_state_ = gtsam::NavState(gtsam::Pose3(), gtsam::Vector3(0,0,0));
    prev_bias_ = gtsam::imuBias::ConstantBias();

    auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Constant(0.1));
    auto vel_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Constant(0.1));
    auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Constant(0.01));

    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), prev_state_.pose(), pose_noise));
    graph_.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), prev_state_.velocity(), vel_noise));
    graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), prev_bias_, bias_noise));

    initial_values_.insert(X(0), prev_state_.pose());
    initial_values_.insert(V(0), prev_state_.velocity());
    initial_values_.insert(B(0), prev_bias_);

    isam_.update(graph_, initial_values_);
    graph_.resize(0);
    initial_values_.clear();
}

void LocalizationNode::optimize(const rclcpp::Time &stamp)
{
    isam_.update(graph_, initial_values_);
    auto result = isam_.calculateEstimate();
    gtsam::Pose3 pose = result.at<gtsam::Pose3>(X(key_));
    gtsam::Vector3 vel = result.at<gtsam::Vector3>(V(key_));
    prev_state_ = gtsam::NavState(pose, vel);
    publishOdometry(pose, vel, stamp);
    graph_.resize(0);
    initial_values_.clear();
}

void LocalizationNode::publishOdometry(const gtsam::Pose3 &pose,
                                        const gtsam::Vector3 &vel,
                                        const rclcpp::Time &stamp)
{
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.child_frame_id = "base_link";

    auto t = pose.translation();
    auto q = pose.rotation().toQuaternion();

    msg.pose.pose.position.x = t.x();
    msg.pose.pose.position.y = t.y();
    msg.pose.pose.position.z = t.z();

    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();

    msg.twist.twist.linear.x = vel.x();
    msg.twist.twist.linear.y = vel.y();
    msg.twist.twist.linear.z = vel.z();

    odom_pub_->publish(msg);
}
