#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import numpy as np
import gtsam

from gtsam import (
    ISAM2,
    NonlinearFactorGraph,
    Values,
    ImuFactor,
    CombinedImuFactor,
    PriorFactorPose3,
    PriorFactorVector,
    PriorFactorConstantBias,
)

from gtsam.symbol_shorthand import X, V, B
from gtsam import Rot3, Pose3, noiseModel

def vector3(x, y, z):
    return np.array([x, y, z], dtype=float)


class ImuISAM2Node(Node):

    def __init__(self):

        super().__init__("imu_isam2_node")

        # subscriber
        self.sub = self.create_subscription(
            Imu,
            "imu/data",
            self.imu_callback,
            100)

        # publisher for visualization
        self.odom_pub = self.create_publisher(
            Odometry,
            "imu/odometry",
            10)

        # gravity
        self.g = 9.81

        # Preintegration parameters
        self.params = gtsam.PreintegrationCombinedParams.MakeSharedU(self.g)

        I = np.eye(3)
        self.params.setAccelerometerCovariance(I * 0.01)
        self.params.setGyroscopeCovariance(I * 0.01)
        self.params.setIntegrationCovariance(I * 0.0001)
        self.bias = gtsam.imuBias.ConstantBias()

        # self.accum = gtsam.PreintegratedCombinedMeasurements(
            # self.params)
        self.accum = gtsam.PreintegratedCombinedMeasurements(self.params, self.bias)

        # ISAM2
        self.isam = ISAM2()

        self.graph = NonlinearFactorGraph()
        self.initialEstimate = Values()

        # state index
        self.i = 0

        
        # last timestamp
        
        self.last_imu_time = None
        self.last_keyframe_time = None
        self.keyframe_dt = 0.1   # 10 Hz state updates

        # initialize first state
        self.initialize_graph()

        self.get_logger().info("IMU ISAM2 Node Initialized")

    def initialize_graph(self):

        pose = gtsam.Pose3()
        vel = vector3(0, 0, 0)

        pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.1]*6))

        vel_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)

        bias_noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)

        self.graph.add(
            PriorFactorPose3(X(0), pose, pose_noise))

        self.graph.add(
            PriorFactorVector(V(0), vel, vel_noise))

        self.graph.add(
            PriorFactorConstantBias(
                B(0), self.bias, bias_noise))

        self.initialEstimate.insert(X(0), pose)
        self.initialEstimate.insert(V(0), vel)
        self.initialEstimate.insert(B(0), self.bias)
        

        self.isam.update(self.graph, self.initialEstimate)

        self.graph.resize(0)
        self.initialEstimate.clear()

    def imu_callback(self, msg):

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_imu_time is None:
            self.last_imu_time = t
            self.last_keyframe_time = t
            return

        dt = t - self.last_imu_time
        self.last_imu_time = t

        if dt <= 0:
            return

        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z])

        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z])

        self.accum.integrateMeasurement(acc, gyro, dt)

        dt_keyframe = t - self.last_keyframe_time

        if dt_keyframe < self.keyframe_dt:
            return

        self.last_keyframe_time = t
        self.i += 1

        # print(f"Keyframe {self.i}")


        factor = gtsam.CombinedImuFactor(
            X(self.i - 1), V(self.i - 1),
            X(self.i),     V(self.i),
            B(0),          B(0),
            self.accum
        )
        self.graph.add(factor)

        result = self.isam.calculateEstimate()

        prev_pose = result.atPose3(X(self.i - 1))
        prev_vel = result.atVector(V(self.i - 1))

        prev_state = gtsam.NavState(prev_pose, prev_vel)

        # Predict new state from IMU
        pred_state = self.accum.predict(prev_state, self.bias)
        pred_pose  = pred_state.pose()
        pred_vel   = pred_state.velocity()

        # Insert predicted state
        self.initialEstimate.insert(X(self.i), pred_pose)
        self.initialEstimate.insert(V(self.i), pred_vel)


         # --- optional: add IMU orientation as prior ---
        imu_orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        R = gtsam.Rot3.Quaternion(
            imu_orientation[3],  # w
            imu_orientation[0],  # x
            imu_orientation[1],  # y
            imu_orientation[2]   # z
        )
        # pose prior: keep translation, replace rotation
        pose_prior = gtsam.Pose3(R, pred_pose.translation())
        orient_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01, 0, 0, 0]))
        self.graph.add(gtsam.PriorFactorPose3(X(self.i), pose_prior, orient_noise))


        self.isam.update(self.graph, self.initialEstimate)

        result = self.isam.calculateEstimate()

        pose = result.atPose3(X(self.i))
        velocity = result.atVector(V(self.i))
        self.bias = result.atConstantBias(B(0))

        self.publish_odometry(pose, velocity, msg.header.stamp)

        self.accum.resetIntegrationAndSetBias(self.bias)
        self.graph.resize(0)
        self.initialEstimate.clear()

    def publish_odometry(self, pose, velocity, stamp):

        odom = Odometry()

        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        t = pose.translation()
        q = pose.rotation().toQuaternion()

        odom.pose.pose.position.x = float(t[0])
        odom.pose.pose.position.y = float(t[1])
        odom.pose.pose.position.z = float(t[2])

        odom.pose.pose.orientation.x = q.x()
        odom.pose.pose.orientation.y = q.y()
        odom.pose.pose.orientation.z = q.z()
        odom.pose.pose.orientation.w = q.w()

        odom.twist.twist.linear.x = float(velocity[0])
        odom.twist.twist.linear.y = float(velocity[1])
        odom.twist.twist.linear.z = float(velocity[2])

        self.odom_pub.publish(odom)


def main():

    rclpy.init()

    node = ImuISAM2Node()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()