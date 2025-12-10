#ifndef NAVBOT_DRIVER_HPP
#define NAVBOT_DRIVER_HPP

#define RAD_S_TO_RPM 9.549 // 60 / (2*PI)

#include "rclcpp/rclcpp.hpp"
#include "navbot_interfaces/msg/multi_values.hpp"

using namespace std::placeholders;

class NavbotDriverNode : public rclcpp::Node 
{
public:
    NavbotDriverNode() : Node("navbot_driver") 
    {
        left_position_ = 0;
        left_velocity_ = 0;
        right_position_ = 0;
        right_velocity_ = 0;
        set_vel_pub_ = this->create_publisher<navbot_interfaces::msg::MultiValues>("set_velocity", 10);
        get_vel_sub_ = this->create_subscription<navbot_interfaces::msg::MultiValues>("get_velocity", 10, std::bind(&NavbotDriverNode::getVelCallback, this, _1));
        get_pos_sub_ = this->create_subscription<navbot_interfaces::msg::MultiValues>("get_position", 10, std::bind(&NavbotDriverNode::getPosCallback, this, _1));
    }

    void setTargetVelocityRadianPerSec(double left_command, double right_command)
    {
        int left_cmd = left_command * RAD_S_TO_RPM;
        int right_cmd = right_command * RAD_S_TO_RPM;

        auto msg = navbot_interfaces::msg::MultiValues();
        msg.left_value = (float)left_cmd;
        msg.right_value = (float)right_cmd;
        set_vel_pub_->publish(msg);
    }

private:
    void getVelCallback(const navbot_interfaces::msg::MultiValues::SharedPtr msg)
    {
        left_velocity_ = msg->left_value;
        right_velocity_ = msg->right_value;
    }

    void getPosCallback(const navbot_interfaces::msg::MultiValues::SharedPtr msg)
    {
        left_position_ = msg->left_value;
        right_position_ = msg->right_value;
    }

    double left_velocity_;
    double right_velocity_;
    double left_position_;
    double right_position_;
    rclcpp::Publisher<navbot_interfaces::msg::MultiValues>::SharedPtr set_vel_pub_;
    rclcpp::Subscription<navbot_interfaces::msg::MultiValues>::SharedPtr get_vel_sub_;
    rclcpp::Subscription<navbot_interfaces::msg::MultiValues>::SharedPtr get_pos_sub_;
};

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MyCustomNode>(); 
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


#endif 
