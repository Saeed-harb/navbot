#include <memory>
#include <string>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines> 
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "navbot_libraries/path_finding.hpp"



using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;

class PathPlannerNode : public rclcpp::Node 
{
public:
    PathPlannerNode() : Node("navbot_controller") 
    {
        // Declare parameters with default values and documentation
        this->declare_parameter("parent_frame", "map");

        route_ = std::make_shared<PathFinding>();

        // Get the values of our parameters
        frame_ = this->get_parameter("parent_frame").as_string();

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();  // map is latched
        qos.reliable();

        // Create a publisher for the /map topic
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos,
        std::bind(&PathPlannerNode::get_map_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filteredS", 10,
        std::bind(&PathPlannerNode::odom_callback, this, std::placeholders::_1));
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&PathPlannerNode::goal_callback, this, std::placeholders::_1));

        // Create a publisher for the /path topic
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "path", 10);

        // Log that we've successfully initialized
        RCLCPP_INFO(this->get_logger(),
        "Path planner initialized with parent frame: '%s'",
        frame_.c_str());
    }

private:

    void get_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Copy the map data 
        map_data_ = msg;
        RCLCPP_WARN(this->get_logger(), "Recieved a new map");
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        // Storing the odom msg
        odom_ = odom_msg->pose.pose.position;
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg) {
        nc::NdArray<int> map_array = get_map_array();

        if (!map_array) {
        RCLCPP_WARN(this->get_logger(), "No map received yet.");
        return;
        }

        int r = map_array.numRows();
        int c = map_array.numCols();

        route_->setGrid(r, c, map_array);

        Pair src = calculate_map_coordinates(odom_.x, odom_.y);
        Pair dest = calculate_map_coordinates(goal_msg->pose.position.x, goal_msg->pose.position.y);

        if (route_->findPath(src, dest)) 
        {
            path_vector_ = route_->getPath();
            publish_path();
        }

        else
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't find a valid path");
        }
    }

    nc::NdArray<int> get_map_array()
    {
        // Check if we received a map message
        if (!map_data_) {
            RCLCPP_WARN(this->get_logger(), "Map is null. Returning empty array.");
            return nc::NdArray<int>();  // empty
        }
        
        // Check if the map actually contains data
        if (map_data_->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Map data vector is empty. Returning empty array.");
            return nc::NdArray<int>();  // empty
        }

        int height = map_data_->info.height;
        int width = map_data_->info.width;

        // Create an NdArray with the correct shape
        nc::NdArray<int> map_array(height, width);

        // Copy data from OccupancyGrid (stored as a flat vector)
        // ROS stores data in row-major order, which matches NumCpp.
        for (int r = 0; r < height; r++){
            for (int c = 0; c < width; c++){
                int index = r * width + c;
                map_array(r, c) = static_cast<int>(map_data_->data[index]);
            }
        }

        return map_array;
    }

    Pair calculate_map_coordinates(double wx, double wy)
    {
        int mx = (wx - map_data_->info.origin.position.x) / map_data_->info.resolution;
        int my = (wy - map_data_->info.origin.position.y) / map_data_->info.resolution;

        // printf("X: %d, Y: %d\n", mx, my);

        return make_pair(my, mx);
    }

    void publish_path()
    {
        // Create a new path message
        nav_msgs::msg::Path path;
        // Set the timestamp to now
        path.header.stamp = this->get_clock()->now();
        // The frame ID should match the frame we want the pose expressed in
        path.header.frame_id = frame_;

        vector<Pair> smoothed_path = smoothPath(path_vector_);

        for (const auto& element : smoothed_path)
        {
        int y = element.first;
        int x = element.second;
        float wx = map_data_->info.origin.position.x + x * map_data_->info.resolution;
        float wy = map_data_->info.origin.position.y + y * map_data_->info.resolution;

        // printf("X: %d, Y: %d\n", x, y);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();  // FIX 1
        pose.header.frame_id = frame_;

        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.w = 1.0;  // Identity quaternion

        path.poses.push_back(pose);

        path.poses.push_back(pose);
        }

        path_pub_->publish(path);
    }

    vector<Pair> smoothPath(const vector<Pair>& path, int samples = 50)
    {
        int N = path.size();
        if (N < 3) return path; // Not enough points for smoothing

        // Create matrix of points (2 x N)
        Eigen::MatrixXd points(2, N);
        for (int i = 0; i < N; i++) {
            points(0, i) = path[i].first;   // y coordinate
            points(1, i) = path[i].second;  // x coordinate
        }

        // Parameter vector (N values evenly spaced between 0 and 1)
        Eigen::RowVectorXd u = Eigen::RowVectorXd::LinSpaced(N, 0.0, 1.0);

        // Fit cubic spline
        auto spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(
            points,
            3,          // cubic spline
            u
        );

        // Sample smoothed points
        vector<Pair> smoothed;
        for (int i = 0; i < samples; i++) {
            double t = double(i) / (samples - 1);

        Eigen::Vector2d pt = spline(t);

        smoothed.emplace_back(
            static_cast<int>(pt(0)),
            static_cast<int>(pt(1))
        );
        }

        return smoothed;
    }

    // Variables 
    std::string frame_;
    std::vector<Pair> path_vector_;
    geometry_msgs::msg::Point odom_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_data_;

    // Shared pointer to PathFinding class
    shared_ptr<PathFinding> route_;

    // ROS infrastructure
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlannerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
