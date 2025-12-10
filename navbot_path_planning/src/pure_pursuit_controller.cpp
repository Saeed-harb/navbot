#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;

class PurePursuitController 
{
public:
    PurePursuitController(double look_ahead_distance)
    {
        look_ahead_distance_ = look_ahead_distance;
    }

    double get_curvature(const Pose & robot_pose) {
        PoseStamped target_pose = get_target_pose(robot_pose);
        double distance = get_distance(target_pose, robot_pose);
        if (distance > 0.001) {
            double dy = target_pose.pose.position.y - robot_pose.position.y;
            return 2.0 * abs(dy) / distance;
        }
        else
            return 0.0;
    }

    void set_path(nav_msgs::msg::Path path) {
        path_ = path;
    }

private:

    PoseStamped get_target_pose(const Pose & robot_pose) {
        PoseStamped target_pose = this->path_.poses.back();
        for (const auto & pose : path_.poses) {
            double distance = sqrt(get_distance(pose, robot_pose));
            if (distance >= look_ahead_distance_) {
                target_pose = pose;
            }
            else
                break; 
        }
        return target_pose;
    }

    double get_distance(const PoseStamped & pose, const Pose & robot_pose) {
        double dx = pose.pose.position.x - robot_pose.position.x;
        double dy = pose.pose.position.y - robot_pose.position.y;
        double distance = pow(dx, 2) + pow(dy, 2);
        return distance;
    }

    double look_ahead_distance_;
    nav_msgs::msg::Path path_;
};

class ControllerNode : public rclcpp::Node 
{
public:
    ControllerNode() : Node("navbot_controller") 
    {
        this->declare_parameter("look_ahead_distance", 0.5);
        this->declare_parameter("max_linear_velocity", 0.3);
        this->declare_parameter("max_angular_velocity", 1.0);

        look_ahead_distance_ = this->get_parameter("look_ahead_distance").as_double();
        max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();

        controller_ = std::make_shared<PurePursuitController>(look_ahead_distance_);

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10,
            std::bind(&ControllerNode::path_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diff_drive_controller/odom", 10,
            std::bind(&ControllerNode::odom_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/pure_pursuit/target", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControllerNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Navbot controller node has been started");
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr path_msg) {
        // Storing the path msg
        path_ = path_msg;
        controller_->set_path(*path_);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        // Storing the odom msg
        odom_ = odom_msg;
    }

    void control_loop() {
        if (!path_ || path_->poses.empty()) 
            return;
        
        if (!odom_) 
            return;

        double curvature = controller_->get_curvature(odom_->pose.pose);

        //  Create and publish the velocity command
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = max_linear_velocity_;
        cmd_vel.angular.z = curvature * max_angular_velocity_;
        cmd_pub_->publish(cmd_vel);
    }

    // Variable to store the path
    std::shared_ptr<PurePursuitController> controller_;
    double look_ahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    nav_msgs::msg::Path::SharedPtr path_;
    nav_msgs::msg::Odometry::SharedPtr odom_;

    // ROS infrastructure
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
