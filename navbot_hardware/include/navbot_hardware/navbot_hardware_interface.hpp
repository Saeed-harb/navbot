#ifndef NAVBOT_HARDWARE_INTERFACE_HPP
#define NAVBOT_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "navbot_interfaces/msg/multi_values.hpp"
#include "navbot_interfaces/msg/velocity.hpp"

#define RAD_S_TO_RPM 9.549 // 60 / (2*PI)

using MultiValues = navbot_interfaces::msg::MultiValues;
using Velocity = navbot_interfaces::msg::Velocity;

namespace navbot_hardware {

class NavbotHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // Lifecycle node override
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // SystemInterface override
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::return_type
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    void setTargetVelocityRadianPerSec(double left_command, double right_command);
    
private:
    double left_velocity_;
    double right_velocity_;
    double left_position_;
    double right_position_;
    rclcpp::Publisher<Velocity>::SharedPtr set_vel_pub_;
    rclcpp::Subscription<Velocity>::SharedPtr get_vel_sub_;
    rclcpp::Subscription<MultiValues>::SharedPtr get_pos_sub_;


}; // class NavbotHardwareInterface

} // namespace navbot_hardware


#endif
