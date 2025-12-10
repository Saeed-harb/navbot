/**
 * @file path_follower.cpp
 * @brief Relay node that converts Twist messages to TwistStamped messages
 *
 * This program subscribes to velocity commands published as Twist messages and
 * republishes them as TwistStamped messages. This is useful when interfacing
 * between different ROS2 nodes that expect different message types for velocity
 * commands.
 *
 * Subscription Topics:
 *     /cmd_vel (geometry_msgs/Twist): Raw velocity commands
 *
 * Publishing Topics:
 *     /mecanum_drive_controller/cmd_vel (geometry_msgs/TwistStamped):
 *     Timestamped velocity commands
 *
 * @author Addison Sears-Collins
 * @date November 30, 2024
 */

// Include necessary header files
#include <memory>  // For smart pointers
#include "rclcpp/rclcpp.hpp"  // Main ROS2 C++ library
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"  // For Twist messages
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"  // For TwistStamped messages
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>

/**
 * @brief A ROS2 node that relays velocity commands between different message types
 *
 * This class creates a node that subscribes to Twist messages and republishes
 * them as TwistStamped messages, adding a timestamp and frame ID.
 */
class PathFollowerNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the PathFollower node
     *
     * Initializes the node, sets up parameters, and creates publishers and subscriptions. 
     */
    PathFollowerNode() : Node("path_follower") {
        // Declare parameters with default values and documentation
        this->declare_parameter("look_ahead_distance", 0.3);
        this->declare_parameter("max_linear_velocity", 0.3);
        this->declare_parameter("max_angular_velocity", 1.0);

        // Get the values of our parameters
        look_ahead_distance_ = this->get_parameter("look_ahead_distance").as_double();
        max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();

        kp_ = 0.5;
        
        // Create a transform buffer to store and look up transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        // Create a transform listener to receive transforms
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create subscription to /cmd_vel topic
        // The '10' represents the queue size - how many messages to store if we can't process them fast enough
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10,
            std::bind(&PathFollowerNode::path_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&PathFollowerNode::odom_callback, this, std::placeholders::_1));
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&PathFollowerNode::goal_callback, this, std::placeholders::_1));

        // Create publishers for /cmd_vel and /pure_pursuit/carrot topics
        // Again, '10' is the queue size for outgoing messages
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        carrot_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/pure_pursuit/carrot", 10);

        // Create a timer for the control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PathFollowerNode::control_loop, this));

        // Log a message indicating the node has started
        RCLCPP_INFO(this->get_logger(), "Path follower node started");
    }

private:
    /**
     * @brief Callback function that handles incoming Twist messages
     *
     * This function is called whenever a new message arrives on the /cmd_vel topic.
     * It creates a new TwistStamped message, copies the velocity data, adds
     * a timestamp and frame ID, and publishes the result.
     *
     * @param msg Shared pointer to the incoming Twist message
     */
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        // Storing the path msg
        path_ = msg;
        RCLCPP_INFO(this->get_logger(), "Recieved a new path");
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        // Storing the path msg
        tf2::Quaternion q_pose(
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w
        );
        current_orientation_ = q_pose.getAngle();
    }  

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg) {
        tf2::Quaternion q_pose(
            goal_msg->pose.orientation.x,
            goal_msg->pose.orientation.y,
            goal_msg->pose.orientation.z,
            goal_msg->pose.orientation.w
        );
        target_orientation_ = q_pose.getAngle();
    }
    
    void control_loop() {
        if (!path_ || path_->poses.empty()) 
            return;
        
        // Get the robot's current pose in the odom frame
        geometry_msgs::msg::TransformStamped robot_pose_transform;
        try {
            // Look up the transform
            robot_pose_transform = tf_buffer_->lookupTransform(
                "odom",
                "base_footprint",
                tf2::TimePointZero // get latest transform
            );   
        }
        catch (const tf2::TransformException & ex) {
            // If we can't get the transform, log it at debug level to avoid spamming
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }

        // Transform plan to robot's frame
        if (!transform_plan(robot_pose_transform.header.frame_id)) {
            RCLCPP_ERROR(this->get_logger(), "Unable to transform Plan in robot's frame");
            return;
        }

        geometry_msgs::msg::PoseStamped robot_pose;
        robot_pose.header.frame_id = robot_pose_transform.header.frame_id;
        robot_pose.pose.position.x = robot_pose_transform.transform.translation.x;
        robot_pose.pose.position.y = robot_pose_transform.transform.translation.y;
        robot_pose.pose.orientation = robot_pose_transform.transform.rotation;

        geometry_msgs::msg::PoseStamped carrot_pose = get_carrot_pose(robot_pose);
        double dx = carrot_pose.pose.position.x - robot_pose.pose.position.x;
        double dy = carrot_pose.pose.position.y - robot_pose.pose.position.y;
        double distance = sqrt(pow(dx, 2) + pow(dy, 2));

        geometry_msgs::msg::Twist cmd_vel;

        if (distance <= 0.1) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = control_angle();
            cmd_pub_->publish(cmd_vel);
            return;
        }

        carrot_pose_pub_->publish(carrot_pose);

        // Calculate the curvature to the look-ahead point
        geometry_msgs::msg::Pose carrot_pose_robot = transformCarrotToRobot(robot_pose, carrot_pose);
        double curvature = get_curvature(carrot_pose_robot);

        //  Create and publish the velocity command
        cmd_vel.linear.x = max_linear_velocity_;
        cmd_vel.angular.z = curvature * max_angular_velocity_;
        cmd_pub_->publish(cmd_vel);
    }

    geometry_msgs::msg::PoseStamped get_carrot_pose(const geometry_msgs::msg::PoseStamped & robot_pose) {
        geometry_msgs::msg::PoseStamped carrot_pose = path_->poses.back();
        std::vector<geometry_msgs::msg::PoseStamped> reversed_poses(path_->poses.rbegin(), path_->poses.rend());
        for (const auto& pose : reversed_poses) {
            double dx = pose.pose.position.x - robot_pose.pose.position.x;
            double dy = pose.pose.position.y - robot_pose.pose.position.y;
            double distance = std::hypot(dx, dy);

            if (distance > this->look_ahead_distance_) {
                carrot_pose = pose;
            }
            else
                break;
        }

        return carrot_pose;
    }

    double get_curvature(const geometry_msgs::msg::Pose & carrot_pose) {
        double carrot_dist = pow(carrot_pose.position.x, 2) + pow(carrot_pose.position.y, 2);
        if (carrot_dist > 0.001) {
            return (2.0 * carrot_pose.position.y) / carrot_dist;
        }
        else {
            return 0.0;
        }
    }

    bool transform_plan(const std::string &target_frame) {
        if (path_->header.frame_id == target_frame) {
            return true;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                target_frame,
                path_->header.frame_id,
                tf2::TimePointZero  // same as rclpy.time.Time()
            );
        }
        catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR(
                this->get_logger(),
                "Couldn't transform plan from %s to %s: %s",
                path_->header.frame_id.c_str(),
                target_frame.c_str(),
                ex.what());
            return false;
        }

        // Transform each pose in the global plan
        for (auto &pose_stamped : path_->poses) {
            geometry_msgs::msg::PoseStamped transformed_pose;
            tf2::doTransform(pose_stamped, transformed_pose, transform);
            pose_stamped = transformed_pose;
        }

        path_->header.frame_id = target_frame;
        return true;
    }

    double control_angle()
    {
        double error = target_orientation_ - current_orientation_;
        double control_signal = kp_ * error;

        geometry_msgs::msg::Twist cmd_vel;
        if (error < 0.1) 
        {
            RCLCPP_INFO(this->get_logger(), "Goal Reached!");
            path_->poses.clear();
            return 0.0;
        }
        RCLCPP_INFO(this->get_logger(), "Control_signal: %f", target_orientation_);
        return control_signal;
    }

    geometry_msgs::msg::Pose transformCarrotToRobot(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const geometry_msgs::msg::PoseStamped& carrot_pose)
    {
        // Convert poses to tf2 transforms
        tf2::Transform robot_tf, carrot_tf;

        // Robot transform
        tf2::Quaternion q_robot(
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z,
            robot_pose.pose.orientation.w
        );
        robot_tf.setOrigin(tf2::Vector3(
            robot_pose.pose.position.x,
            robot_pose.pose.position.y,
            robot_pose.pose.position.z
        ));
        robot_tf.setRotation(q_robot);

        // Carrot transform
        tf2::Quaternion q_carrot(
            carrot_pose.pose.orientation.x,
            carrot_pose.pose.orientation.y,
            carrot_pose.pose.orientation.z,
            carrot_pose.pose.orientation.w
        );
        carrot_tf.setOrigin(tf2::Vector3(
            carrot_pose.pose.position.x,
            carrot_pose.pose.position.y,
            carrot_pose.pose.position.z
        ));
        carrot_tf.setRotation(q_carrot);

        // Transform carrot pose into robot frame
        tf2::Transform carrot_in_robot_tf = robot_tf.inverseTimes(carrot_tf);

        // Convert result back to PoseStamped
        geometry_msgs::msg::Pose carrot_in_robot;

        carrot_in_robot.position.x = carrot_in_robot_tf.getOrigin().x();
        carrot_in_robot.position.y = carrot_in_robot_tf.getOrigin().y();
        carrot_in_robot.position.z = carrot_in_robot_tf.getOrigin().z();

        tf2::Quaternion q_out = carrot_in_robot_tf.getRotation();
        carrot_in_robot.orientation.x = q_out.x();
        carrot_in_robot.orientation.y = q_out.y();
        carrot_in_robot.orientation.z = q_out.z();
        carrot_in_robot.orientation.w = q_out.w();

        return carrot_in_robot;
    }

    // Variable to store the path
    float kp_;
    double current_orientation_;
    double target_orientation_;
    double look_ahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    nav_msgs::msg::Path::SharedPtr path_;
    nav_msgs::msg::Odometry::SharedPtr odom_;

    // ROS infrastructure
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;        
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; 
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief Main function that initializes and runs the node
 *
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int Exit status
 */
int main(int argc, char* argv[]) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create an instance of our node
    auto node = std::make_shared<PathFollowerNode>();

    // Spin the node, making it respond to incoming messages
    rclcpp::spin(node);

    // Clean up ROS2 resources
    rclcpp::shutdown();

    return 0;
}
