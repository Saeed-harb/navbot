#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from ahrs.filters import Madgwick
import numpy as np

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.subscriber_ = self.create_subscription(Float64MultiArray, '/imu', self.subscriber_callback, 10)

        self.filter_ = Madgwick(beta=0.01)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # initial quaternion

        self.prev_time = None
        self.integrated_angle_z = 0.0

    def subscriber_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 9:
            return  # invalid message
        now = self.get_clock().now().nanoseconds * 1e-9  # seconds
        if self.prev_time is None:
            self.prev_time = now
        dt = now - self.prev_time
        self.prev_time = now

        ax, ay, az, gx, gy, gz, mx, my, mz = msg.data
        # Madgwick expects rad/sec for gyro
        gx, gy, gz = np.deg2rad([gx, gy, gz])

        # assume incoming gx is in degrees/sec — try this first
        # angle_delta_deg = gy * dt    # if gz is deg/s -> deg
        # self.integrated_angle_z += angle_delta_deg

        # # print diagnostic
        # self.get_logger().info(f"gz={gz:.3f}  dt={dt:.4f}  integrated_z_deg={self.integrated_angle_z:.2f}")

        center = np.array([-128.49672722, -220.38466704, 314.12149445])
        M = np.array([[ 1.09016962,  0.05261095, -0.03531870],
              [ 0.05261095,  0.92690813,  0.07389396],
              [-0.03531870,  0.07389396,  1.01697901]])

        # raw mag from message: mx,my,mz
        raw = np.array([mx, my, mz])
        mag_cal = M.dot(raw - center)

        # Madgwick expects nanoTesla for mag
        mag_cal_nT = mag_cal * 1000.0
        # mx *= 1000
        # my *= 1000
        # mz *= 1000
        # raw = np.array([mx, my, mz])

        # Update the filter
        self.q = self.filter_.updateIMU(self.q, np.array([gx, gy, gz]), np.array([ax, ay, az]), dt)
        # self.q = self.filter_.updateMARG(self.q, np.array([gx, gy, gz]), np.array([ax, ay, az]), mag_cal_nT, dt)
        # self.q = self.q / np.linalg.norm(self.q)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Orientation (from quaternion)
        imu_msg.orientation.w =  self.q[0]
        imu_msg.orientation.x =  self.q[1]
        imu_msg.orientation.y =  self.q[2]
        imu_msg.orientation.z =  self.q[3]

        # Angular velocity
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        # Linear acceleration
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        self.publisher_.publish(imu_msg)

    # except Exception as e:
    #     self.get_logger().warn(f"Error parsing line: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
