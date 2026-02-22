#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import gtsam

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from gtsam.symbol_shorthand import X, V, B


class GtsamImuNode(Node):

    def __init__(self):

        super().__init__("gtsam_imu_node")

        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            "/alpha_rise/imu/odometry",
            10
        )

        # Subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            "/alpha_rise/ekf/imu/data",
            self.imu_callback,
            200
        )

        # ISAM2
        self.isam = gtsam.ISAM2()

        self.graph = gtsam.NonlinearFactorGraph()
        self.initial = gtsam.Values()

        # IMU parameters (ENU frame)
        params = gtsam.PreintegrationCombinedParams.MakeSharedU(9.81)

        params.setAccelerometerCovariance(np.eye(3) * 0.01)
        params.setGyroscopeCovariance(np.eye(3) * 0.001)
        params.setIntegrationCovariance(np.eye(3) * 1e-8)

        params.setBiasAccCovariance(np.eye(3) * 1e-4)
        params.setBiasOmegaCovariance(np.eye(3) * 1e-5)
        params.setBiasAccOmegaInit(np.eye(6) * 1e-5)
        # gravity ENU
        # params.setGravity(np.array([0, 0, -9.81]))
        params = gtsam.PreintegrationCombinedParams.MakeSharedU(9.81)

        self.bias = gtsam.imuBias.ConstantBias()

        self.pim = gtsam.PreintegratedCombinedMeasurements(
            params,
            self.bias
        )

        # noise models

        self.pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.01, 0.01, 0.01, 1000, 1000, 1000])
        )

        self.bias_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.ones(6) * 1e-3
        )

        self.vel_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)

        # state index
        self.k = 0

        self.prev_time = None

        self.initialized = False
        self.latest_pose = None
        self.latest_vel = None

        self.publish_rate = 10.0  # Hz

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_odometry_timer
        )


    def imu_callback(self, msg: Imu):

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_time is None:
            self.prev_time = t
            return

        dt = t - self.prev_time
        self.prev_time = t

        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # integrate imu
        self.pim.integrateMeasurement(
            accel,
            gyro,
            dt
        )

        # get orientation from AHRS
        q = msg.orientation

        rot = gtsam.Rot3.Quaternion(
            q.w,
            q.x,
            q.y,
            q.z
        )

        if not self.initialized:

            self.initialize(rot)
            self.initialized = True
            return

        self.add_imu_factor()
        self.add_orientation_factor(rot)

        self.optimize()

        # self.publish_odometry(msg.header.stamp)

        self.pim.resetIntegrationAndSetBias(self.bias)

        self.k += 1


    def initialize(self, rot):

        pose = gtsam.Pose3(
            rot,
            gtsam.Point3(0, 0, 0)
        )

        vel = np.zeros(3)

        self.initial.insert(X(0), pose)
        self.initial.insert(V(0), vel)
        self.initial.insert(B(0), self.bias)

        self.graph.add(
            gtsam.PriorFactorPose3(
                X(0),
                pose,
                self.pose_noise
            )
        )

        self.graph.add(
            gtsam.PriorFactorVector(
                V(0),
                vel,
                self.vel_noise
            )
        )

        self.graph.add(
            gtsam.PriorFactorConstantBias(
                B(0),
                self.bias,
                self.bias_noise
            )
        )

        self.isam.update(
            self.graph,
            self.initial
        )

        self.graph.resize(0)
        self.initial.clear()


    def add_imu_factor(self):

        factor = gtsam.CombinedImuFactor(
            X(self.k),
            V(self.k),
            X(self.k + 1),
            V(self.k + 1),
            B(self.k),
            B(self.k + 1),
            self.pim
        )

        self.graph.add(factor)

        result = self.isam.calculateEstimate()

        pose = result.atPose3(X(self.k))
        vel = result.atVector(V(self.k))

        self.initial.insert(X(self.k + 1), pose)
        self.initial.insert(V(self.k + 1), vel)
        self.initial.insert(B(self.k + 1), self.bias)


    def add_orientation_factor(self, rot):

        pose = gtsam.Pose3(
            rot,
            gtsam.Point3(0, 0, 0)
        )

        self.graph.add(
            gtsam.PriorFactorPose3(
                X(self.k + 1),
                pose,
                self.pose_noise
            )
        )


    # def optimize(self):

    #     self.isam.update(
    #         self.graph,
    #         self.initial
    #     )

    #     self.graph.resize(0)
    #     self.initial.clear()

    def optimize(self):

        self.isam.update(self.graph, self.initial)

        result = self.isam.calculateEstimate()

        self.latest_pose = result.atPose3(X(self.k))
        self.latest_vel = result.atVector(V(self.k))

        self.graph.resize(0)
        self.initial.clear()


    def publish_odometry_timer(self):

        if self.latest_pose is None:
            return

        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        t = self.latest_pose.translation()
        q = self.latest_pose.rotation().toQuaternion()

        odom.pose.pose.position.x = float(t[0])
        odom.pose.pose.position.y = float(t[1])
        odom.pose.pose.position.z = float(t[2])

        odom.pose.pose.orientation.x = q.x()
        odom.pose.pose.orientation.y = q.y()
        odom.pose.pose.orientation.z = q.z()
        odom.pose.pose.orientation.w = q.w()    


        odom.twist.twist.linear.x = self.latest_vel[0]
        odom.twist.twist.linear.y = self.latest_vel[1]
        odom.twist.twist.linear.z = self.latest_vel[2]

        self.odom_pub.publish(odom)


def main():

    rclpy.init()

    node = GtsamImuNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()