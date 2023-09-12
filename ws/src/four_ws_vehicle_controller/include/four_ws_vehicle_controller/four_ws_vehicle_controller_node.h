#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace four_ws_vehicle_controller {
class four_ws_vehicle_controller_node : public rclcpp::Node
{
  public:
    four_ws_vehicle_controller_node();

  private:


    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _position_controller_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _velocity_controller_publisher;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _robot_command_subscriber_;

    const std::string _robot_command_topic_name = "/robot_command";
    const std::string _position_controller_topic_name = "/forward_position_controller/commands";
    const std::string _velocity_controller_topic_name = "/forward_velocity_controller/commands";

    void _on_robot_command_callback(const geometry_msgs::msg::Twist& msg);

};

} //namespace four_ws_vehicle_controller