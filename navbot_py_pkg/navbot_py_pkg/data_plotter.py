# #!/usr/bin/env python3
import rclpy
from math import sin
from rclpy.node import Node
from navbot_interfaces.msg import Velocity
import matplotlib.pyplot as plt
import time

class RPMPlotter(Node):
    def __init__(self):
        super().__init__('rpm_plotter')

        # Subscriptions
        self.get_vel_sub_ = self.create_subscription(Velocity, 'get_velocity', self.get_velocity_callback, 10)
        self.set_vel_sub_ = self.create_subscription(Velocity, 'set_velocity', self.set_velocity_callback, 10)
        # self.set_vel_pub_ = self.create_publisher(Velocity, 'set_velocity', 10)
        
        # Data lists
        self.time_data = []
        self.left_rpm_data = []
        self.right_rpm_data = []
        self.left_target_data = []
        self.right_target_data = []
        self.left_target_vel_ = 0.0
        self.right_target_vel_ = 0.0
        self.start_time = time.time()

        # Plot setup
        plt.ion()
        self.fig, (self.ax_right, self.ax_left) = plt.subplots(2, 1, sharex=True)


        self.line_actual_left, = self.ax_left.plot([], [], label='Current RPM Left', color='blue')
        self.line_target_left, = self.ax_left.plot([], [], label='Target RPM Left', color='red', linestyle='--')
        self.ax_left.set_xlabel('Time (s)')
        self.ax_left.set_ylabel('Left RPM')
        self.ax_left.legend()

        self.line_actual_right, = self.ax_right.plot([], [], label='Current RPM Right', color='green')
        self.line_target_right, = self.ax_right.plot([], [], label='Target RPM Right', color='orange', linestyle='--')
        self.ax_right.set_xlabel('Time (s)')
        self.ax_right.set_ylabel('Right RPM')
        self.ax_right.legend()
        self.window = 10  # seconds of data to show

    def get_velocity_callback(self, msg: Velocity):
        t = time.time() - self.start_time
        right_target_vel_ = self.right_target_vel_
        left_target_vel_ = self.left_target_vel_
        self.time_data.append(t)
        self.left_rpm_data.append(msg.left_velocity)
        self.right_rpm_data.append(msg._right_velocity)
        self.left_target_data.append(left_target_vel_)
        self.right_target_data.append(right_target_vel_)

        # Limit window
        while self.time_data[-1] - self.time_data[0] > self.window:
            self.time_data.pop(0)
            self.left_rpm_data.pop(0)
            self.right_rpm_data.pop(0)
            self.left_target_data.pop(0)
            self.right_target_data.pop(0)

        # Update plots
        self.line_actual_left.set_data(self.time_data, self.left_rpm_data)
        self.line_target_left.set_data(self.time_data, self.left_target_data)
        self.line_actual_right.set_data(self.time_data, self.right_rpm_data)
        self.line_target_right.set_data(self.time_data, self.right_target_data)

        self.ax_left.set_xlim(max(0, t - self.window), t)
        self.ax_left.set_ylim(min(self.left_rpm_data + self.left_target_data) - 10,
                         max(self.left_rpm_data + self.left_target_data) + 10)
        self.ax_right.set_xlim(max(0, t - self.window), t)
        self.ax_right.set_ylim(min(self.right_rpm_data + self.right_target_data) - 10,
                         max(self.right_rpm_data + self.right_target_data) + 10)
        plt.draw()
        plt.pause(0.01)

    def set_velocity_callback(self, msg: Velocity):
        self.left_target_vel_ = msg.left_velocity
        self.right_target_vel_ = msg.right_velocity

    # def set_velocity_pub(self, left_vel, right_vel):
    #     msg = Velocity()
    #     msg.left_velocity = left_vel
    #     msg.right_velocity = right_vel
    #     self.set_vel_pub_.publish(msg)
    #     self.left_target_vel_ = left_vel
    #     self.right_target_vel_ = right_vel

def main():
    rclpy.init()
    node = RPMPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    plt.ioff()
    plt.show()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
