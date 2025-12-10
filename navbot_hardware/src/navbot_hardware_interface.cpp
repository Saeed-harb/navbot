#include "navbot_hardware/navbot_hardware_interface.hpp"

namespace navbot_hardware {

hardware_interface::CallbackReturn NavbotHardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;
    left_position_ = 0;
    left_velocity_ = 0;
    right_position_ = 0;
    right_velocity_ = 0;

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NavbotHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    
    auto getVelCallback = [this] (const Velocity::SharedPtr msg) -> void
    {
        left_velocity_ = msg->left_velocity;
        right_velocity_ = msg->right_velocity;
    };
    auto getPosCallback = [this] (const MultiValues::SharedPtr msg) -> void
    {
        left_position_ = msg->left_value;
        right_position_ = msg->right_value;
    };

    get_vel_sub_ = get_node()->create_subscription<Velocity>(
        "/get_velocity", 10, getVelCallback);
    get_pos_sub_ = get_node()->create_subscription<MultiValues>(
        "/get_position", 10, getPosCallback); 
    set_vel_pub_ = get_node()->create_publisher<Velocity>(
        "/set_velocity", 10);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NavbotHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    set_state("base_left_wheel_joint/velocity", 0.0);
    set_state("base_right_wheel_joint/velocity", 0.0);
    set_state("base_left_wheel_joint/position", 0.0);
    set_state("base_right_wheel_joint/position", 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NavbotHardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    // driver_->deactivate(left_motor_id_);
    // driver_->deactivate(right_motor_id_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type NavbotHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    // if (abs(left_vel_) < 0.03) { left_vel = 0.0; }
    // if (abs(right_vel) < 0.03) { right_vel = 0.0; }
    set_state("base_left_wheel_joint/velocity", left_velocity_);
    set_state("base_right_wheel_joint/velocity", right_velocity_);
    set_state("base_left_wheel_joint/position", left_position_);
    set_state("base_right_wheel_joint/position", right_position_);
    // RCLCPP_INFO(get_logger(), "left vel: %lf, right vel: %lf, left pos: %lf, right pos: %lf",
    //          left_vel, right_vel, get_state("base_left_wheel_joint/position"), get_state("base_right_wheel_joint/position"));
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type NavbotHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    double left_cmd = get_command("base_left_wheel_joint/velocity");
    double right_cmd =get_command("base_right_wheel_joint/velocity");
    setTargetVelocityRadianPerSec(left_cmd, right_cmd);
    // RCLCPP_INFO(get_logger(), "left vel: %lf, right vel: %lf", get_command("base_left_wheel_joint/velocity"), 
    //                     get_command("base_right_wheel_joint/velocity"));
    return hardware_interface::return_type::OK;
}

void NavbotHardwareInterface::setTargetVelocityRadianPerSec(double left_command, double right_command)
    {
        int left_cmd = left_command * RAD_S_TO_RPM;
        int right_cmd = right_command * RAD_S_TO_RPM;

        // int left_cmd = left_command;
        // int right_cmd = right_command;

        auto msg = Velocity();
        msg.left_velocity = left_cmd;
        msg.right_velocity = right_cmd;
        set_vel_pub_->publish(msg);
    }

} // namespace navbot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(navbot_hardware::NavbotHardwareInterface, hardware_interface::SystemInterface)
