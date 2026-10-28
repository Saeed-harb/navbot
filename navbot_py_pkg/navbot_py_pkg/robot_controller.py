#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from navbot_interfaces.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from navbot_py_pkg.my_library.pid_class import PID
import matplotlib.pyplot as plt
import tf_transformations

# === PID gains ===
KP_DIST = 1.0
KI_DIST = 0.0
KD_DIST = 0.0

KP_ANGLE = 2.2
KI_ANGLE = 0.0
KD_ANGLE = 1.7

CONTROL_RATE = 0.02  # 20 Hz (seconds per loop)

# === Robot geometry ===
RADIUS = 0.046  # wheel radius in meters


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__("robot_controller")

        # --- Initialize ---
        self.pose_ = None
        self.target_ = None
        self.waypoints_ = []

        self.pid_distance = PID(KP_DIST, KI_DIST, KD_DIST)
        self.pid_angle = PID(KP_ANGLE, KI_ANGLE, KD_ANGLE)

        # --- ROS interfaces ---
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # self.pose_sub = self.create_subscription(Pose, "/odom", self.callback_pose, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)

        # --- Control loop ---
        self.timer = self.create_timer(CONTROL_RATE, self.control_loop)

        # --- Live plotting ---
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.time_data, self.angle_data, self.target_angle_data = [], [], []
        self.start_time = time.time()
        self.window = 30

        print("✅ Robot controller initialized")

    # def callback_pose(self, msg):
    #     self.pose_ = msg

    def odom_callback(self, msg: Odometry):
        self.pose_ = Pose()
        self.pose_.x = msg.pose.pose.position.x
        self.pose_.y = msg.pose.pose.position.y

        q_x, q_y, q_z, q_w = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        e_x, e_y, e_z = tf_transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])
        
        self.pose_.theta = e_z
        print(e_z)

    def set_target(self, waypoints):
        """Set multiple waypoints."""
        self.waypoints_ = waypoints
        if waypoints:
            self.target_ = Pose()
            self.target_.x, self.target_.y = self.waypoints_.pop(0)
            print(f"🎯 Target set to ({self.target_.x:.2f}, {self.target_.y:.2f})")

    def calculate_error(self):
        """Compute distance and heading error."""
        dx = self.target_.x - self.pose_.x
        dy = self.target_.y - self.pose_.y
        distance = math.sqrt(dx**2 + dy**2)

        goal_theta = math.atan2(dy, dx)
        angle_error = goal_theta - self.pose_.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # wrap [-pi, pi]

        return distance, angle_error, goal_theta

    def control_loop(self):
        if self.pose_ is None or self.target_ is None:
            return

        distance, angle_error, goal_theta = self.calculate_error()

        # --- Record data for plotting ---
        t = time.time() - self.start_time
        self.time_data.append(t)
        self.angle_data.append(self.pose_.theta)
        self.target_angle_data.append(goal_theta)
        if self.time_data[-1] - self.time_data[0] > self.window:
            self.time_data.pop(0)
            self.angle_data.pop(0)
            self.target_angle_data.pop(0)
        self.update_plot()

        # --- Compute control ---
        dt = CONTROL_RATE
        angular_z = self.pid_angle.compute(angle_error, dt)

        # Turn in place if angle error large
        if abs(angle_error) > 0.2:
            linear_x = 0.0
        else:
            linear_x = self.pid_distance.compute(distance, dt)

        # Clamp speeds
        linear_x = max(min(linear_x, 0.3), -0.3)
        angular_z = max(min(angular_z, 1.0), -1.0)

        # --- Publish ---
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(cmd)

        self.get_logger().info(f"D={distance:.3f}  A={angle_error:.3f}  v={linear_x:.2f}  w={angular_z:.2f}")

        # --- Target reached ---
        if distance < 0.05:
            # Stop the robot
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

            # Pause at waypoint for N seconds
            pause_time = 3.5  # seconds
            print(f"🕒 Pausing for {pause_time} seconds at waypoint...")
            time.sleep(pause_time)

            # Move to the next waypoint
            if self.waypoints_:
                self.target_.x, self.target_.y = self.waypoints_.pop(0)
                print(f"🎯 New target: ({self.target_.x:.2f}, {self.target_.y:.2f})")
            else:
                print("✅ All waypoints reached.")
                self.cmd_vel_pub.publish(cmd)
                self.timer.cancel()

    def update_plot(self):
        self.ax.clear()
        self.ax.plot(self.time_data, self.angle_data, label='Current θ', color='blue')
        self.ax.plot(self.time_data, self.target_angle_data, label='Target θ', color='red', linestyle='--')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Angle (rad)')
        self.ax.legend()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    node.set_target([
        [0.5, 0.0],
        [0.5, 0.5],
        [1.0, 1.0],
        [0.0, 0.0]
    ])
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

