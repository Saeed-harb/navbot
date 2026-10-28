#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
import tf_transformations


class FollowPathClientNode(Node):
    def __init__(self):
        super().__init__("follow_path_client")
        self.path_publisher_ = self.create_publisher(Path, '/plan', 10)
        self.follow_path_client_ = ActionClient(self, FollowPath, "follow_path")

    def send_goal(self):
        # Wait for the server
        self.follow_path_client_.wait_for_server()

        # Create a goal
        goal = FollowPath.Goal()
        goal.path = self.generate_path()
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "general_goal_checker"

        # Send the goal
        self.get_logger().info("Sending path to follow")
        self.follow_path_client_. \
            send_goal_async(goal, feedback_callback=self.goal_feedback_callback). \
                add_done_callback(self.goal_response_callback)
        
        # Send a cancel request 2 seconds later
        # self.timer_ = self.create_timer(2.0, self.cancel_goal)

    # def cancel_goal(self):
    #     self.get_logger().info("Send a cancel request")
    #     self.goal_handle_.cancel_goal_async()
    #     self.timer_.cancel()

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
           self.get_logger().info("Goal got accepted")
           self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal got rejected")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("Result: " + str(result.error_msg))

    def goal_feedback_callback(self, feedback_msg):
        remaining_distance = feedback_msg.feedback.distance_to_goal
        self.get_logger().info("Got feedback: " + str(remaining_distance))

    def generate_path(self):
        poses = []

        for i in range(1, 20):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            x = i / 10.0
            y = 2.0 * math.sin(x)

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            yaw = math.atan2(2.0 * math.cos(x), 1.0)

            q = tf_transformations.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            poses.append(pose)

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = poses

        self.path_publisher_.publish(path)
        return path

def main(args=None):
    rclpy.init(args=args)
    node = FollowPathClientNode()
    node.send_goal()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
