# #!/usr/bin/env python3
# import math
# import time
# import rclpy
# import numpy as np
# from functools import partial
# from rclpy.node import Node
# from navbot_interfaces.msg import Pose
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from navbot_interfaces.msg import Velocity
# import matplotlib.pyplot as plt
# import time
# # from my_robot_interfaces.msg import Turtle
# # from my_robot_interfaces.msg import TurtleArray
# # from my_robot_interfaces.srv import CatchTurtle

# kp_distance = 0.5
# ki_distance = 0
# kd_distance = 0
# kp_angle = 0.75
# ki_angle = 0.0
# kd_angle = 0.012 # 0.6
# h = 0.005

# radius = 0.046

# class PIDControlClass:
#     def __init__(self, kP, kI, kD, delayP):
#         self.Kp = kP
#         self.Ki = kI
#         self.Kd = kD

#         self.h = delayP

#         self.errorK = 0.0
#         self.errorKm1 = 0.0
#         self.errorKm2 = 0.0
#         self.controlK = 0.0
#         self.controlKm1 = 0.0

#         self.a1 = self.Kp + self.Ki*self.h + self.Kd/self.h
#         self.a2 = -(self.Kp + 2*self.Kd/self.h)
#         self.a3 = self.Kd/self.h

#     def PIDControllerInitialize(self, error):
#         self.errorK = error
#         self.errorKm1 = self.errorK
#         self.errorKm2 = self.errorK

#     def ComputeControlAction(self, error):
#         self.errorK = error

#         if abs(self.errorK) < 0.05:
#             self.controlK = 0
            
#         else:
#             self.controlK = self.controlKm1 + self.a1*self.errorK + self.a2*self.errorKm1 + self.a3*self.errorKm2

#             if self.controlK > 150:
#                 self.controlK = 150
#             if self.controlK < -150:
#                 self.controlK = -150

#         self.controlKm1 = self.controlK
#         self.errorKm2 = self.errorKm1
#         self.errorKm1 = self.errorK
#         # print(str(self.controlK))
#         return self.controlK

# class RobotControllerNode(Node):
#     def __init__(self):
#         super().__init__("robot_controller")
#         self.pose_: Pose = None
#         self.target_: Pose = None
#         self.PIDControllerDistance = PIDControlClass(kp_distance, ki_distance, kd_distance, h)
#         self.PIDControllerAngle = PIDControlClass(kp_angle, ki_angle, kd_angle, h)
#         self.PIDControllerDistance.PIDControllerInitialize(0)
#         self.PIDControllerAngle.PIDControllerInitialize(0)
#         self.cmd_vel_publisher_ = self.create_publisher(
#             Twist, "/cmd_vel", 10)
#         self.set_vel_publisher_ = self.create_publisher(
#             Velocity, "/set_velocity", 10)
#         self.pose_subscriber_ = self.create_subscription(
#             Pose, "/odom", self.callback_pose, 10)
#         # self.control_loop_timer_ = self.create_timer(
#         #     h, self.control_loop)
#         self.angle_control_loop_timer_ = self.create_timer(
#             h, self.angle_control_loop)
#         self.distance_control_loop_timer_ = self.create_timer(
#             h, self.distance_control_loop, autostart=False)
        
#         # Data lists
#         self.waypoints_ = []
#         self.time_data = []
#         self.angle_data = []
#         self.target_angle_data = []
#         self.target_angle = 0.0
#         self.start_time = time.time()

#         # Plot setup
#         plt.ion()
#         self.fig, self.ax_ = plt.subplots(sharex=True)

#         self.line_actual_angle = self.ax_.plot([], [], label='Current Angle', color='blue')
#         self.line_target_angle = self.ax_.plot([], [], label='Target Angle', color='red', linestyle='--')
#         self.ax_.set_xlabel('Time (s)')
#         self.ax_.set_ylabel('Radian')
#         self.ax_.legend()
#         self.window = 30  # seconds of data to show

#     def callback_pose(self, pose: Pose):
#         self.pose_ = pose
#         self.draw()

#     def control_loop(self):
#         if self.pose_ == None or self.target_ == None:
#             return
        
#         distance, angle = self.calculate_error()
#         self.get_logger().info("Distance: " + str(distance) + ", Angle: " + str(angle))

#         if distance < 0.01:
#             angle = 0
#             v = 0
#         else:
#             v = distance * kp_distance

#         if v > 230:
#             v = 230
#         if v < -230:
#             v = -230 

        

#         if angle > math.pi:
#             angle -= 2*math.pi
#         elif angle < -math.pi:
#             angle += 2*math.pi

#         cmd = Twist()

#         # position
#         # print("Velocity: ", v)
#         w = float(self.PIDControllerAngle.ComputeControlAction(angle))
#         # print("V: ", v)
#         # print("W: ", w)
#         vR = v + w
#         vL = v - w

#         # print("vL: ", vL)
#         # print("vR: ", vR)

#         # cmd.right_velocity = int((vR / radius))
#         # cmd.left_velocity = int((vL / radius))

#         # self.set_vel_publisher_.publish(cmd)

#         # cmd = Twist()

#         # # position
#         # cmd.linear.x = float(self.PIDControllerDistance.ComputeControlAction(distance))
#         # cmd.angular.z = float(self.PIDControllerAngle.ComputeControlAction(angle))
#         # cmd.linear.x = float(self.PIDControllerDistance.ComputeControlAction(distance))
#         # cmd.angular.z = float(self.PIDControllerAngle.ComputeControlAction(angle))

#         cmd.linear.x = float(v)
#         cmd.angular.z = float(w)

#         self.cmd_vel_publisher_.publish(cmd)

#     def angle_control_loop(self):
#         if self.pose_ == None or self.target_ == None:
#             return
        
#         print("In angle loop")
        
#         distance, angle = self.calculate_error()
#         self.get_logger().info("Distance: " + str(distance) + ", Angle: " + str(angle))

#         if angle > math.pi:
#             angle -= 2*math.pi
#         elif angle < -math.pi:
#             angle += 2*math.pi

#         cmd = Twist()

#         w = self.PIDControllerAngle.ComputeControlAction(angle)
#         cmd.angular.z = float(w)

#         if w == 0.0:
#             self.angle_control_loop_timer_.cancel()
#             self.distance_control_loop_timer_.reset()
#             time.sleep(1)

#         self.cmd_vel_publisher_.publish(cmd)


#     def distance_control_loop(self):
#         if self.pose_ == None or self.target_ == None:
#             return
#         print("In distance loop")
#         distance, angle = self.calculate_error()
#         self.get_logger().info("Distance: " + str(distance) + ", Angle: " + str(angle))

#         if distance < 0.01:
#             v = 0
#             self.distance_control_loop_timer_.cancel()
#             self.angle_control_loop_timer_.reset()
#             self.target_.x = self.waypoints_[0][0]
#             self.target_.y = self.waypoints_[0][1]
#             self.waypoints_.pop(0)
#             time.sleep(1)
#             print("Target: (" + str(self.target_.x) + ", " + str(self.target_.y) + ")")
            
#         else:
#             v = distance * kp_distance

#         if v > 230:
#             v = 230
#         if v < -230:
#             v = -230

#         print(v)

#         cmd = Twist()
#         cmd.linear.x = float(v)

#         self.cmd_vel_publisher_.publish(cmd)

    
#     def calculate_error(self):
#         dist_x = self.target_.x - self.pose_.x
#         dist_y = self.target_.y - self.pose_.y
#         distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

#         goal_theta = math.atan2(dist_y, dist_x)
#         self.target_angle = goal_theta
#         print("Goal theta: ", goal_theta)
#         # print("Current theta: ", self.pose_.theta)
#         angle = goal_theta - self.pose_.theta
#         # angle = self.target_angle - self.pose_.theta

#         angle = math.atan2(math.sin(angle), math.cos(angle))

#         return distance, angle    
    

#     def set_target(self, waypoints):
#         self.waypoints_ = waypoints
#         self.target_ = Pose()
#         self.target_.x = self.waypoints_[0][0]
#         self.target_.y = self.waypoints_[0][1]
#         print("Target: (" + str(self.target_.x) + ", " + str(self.target_.y) + ")")
#         self.waypoints_.pop(0)

#     def draw(self):
#         t = time.time() - self.start_time
#         self.time_data.append(t)
#         self.angle_data.append(self.pose_.theta)
#         self.target_angle_data.append(self.target_angle)

#         # Limit window
#         while self.time_data[-1] - self.time_data[0] > self.window:
#             self.time_data.pop(0)
#             self.angle_data.pop(0)
#             self.target_angle_data.pop(0)

#         # Update plots
#         self.line_actual_angle[0].set_data(self.time_data, self.angle_data)
#         self.line_target_angle[0].set_data(self.time_data, [self.target_angle] * len(self.time_data))
#         # self.line_target_angle[0].set_data(self.time_data, [self.target_angle] * len(self.time_data))


#         self.ax_.set_xlim(max(0, t - self.window), t)
#         # all_angles = self.angle_data + [self.target_angle]
#         all_angles = self.angle_data + self.target_angle_data
#         self.ax_.set_ylim(min(all_angles) - 0.2, max(all_angles) + 0.2)

#         plt.draw()
#         plt.pause(0.01)


# def main(args=None):
#     rclpy.init(args=args)
#     node = RobotControllerNode()
#     node.set_target([[0.5, 0], [0.5, 0.5], [1, 1]])
#     # node.set_target(0, 0)
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()




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

