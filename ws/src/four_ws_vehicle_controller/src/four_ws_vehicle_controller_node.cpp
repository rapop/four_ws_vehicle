#include <four_ws_vehicle_controller/four_ws_vehicle_controller_node.h>

#include <std_msgs/msg/float64_multi_array.hpp>

namespace four_ws_vehicle_controller
{

    four_ws_vehicle_controller_node::four_ws_vehicle_controller_node()
        : Node("four_ws_vehicle_controller_node")
    {
        const auto qos = rclcpp::QoS(1000);

        _robot_command_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            _robot_command_topic_name, qos, 
            std::bind(&four_ws_vehicle_controller_node::_on_robot_command_callback, this, std::placeholders::_1));

        _position_controller_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        (_position_controller_topic_name, qos);
        _velocity_controller_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        (_velocity_controller_topic_name, qos);
    }

    void four_ws_vehicle_controller_node::_on_robot_command_callback(const geometry_msgs::msg::Twist &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received robot command msg: msg.angular.z %.2f and msg.linear.x %.2f", 
        msg.angular.z, msg.linear.x);

        std_msgs::msg::Float64MultiArray position_controller_data;
        std_msgs::msg::Float64MultiArray velocity_controller_data;

        position_controller_data.data.push_back(msg.angular.z);
        position_controller_data.data.push_back(msg.angular.z);

        velocity_controller_data.data.push_back(msg.linear.x);
        velocity_controller_data.data.push_back(msg.linear.x);
        velocity_controller_data.data.push_back(msg.linear.x);
        velocity_controller_data.data.push_back(msg.linear.x);

        _position_controller_publisher->publish(position_controller_data);
        _velocity_controller_publisher->publish(velocity_controller_data);
    }

} // namespace four_ws_vehicle_controller

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<four_ws_vehicle_controller::four_ws_vehicle_controller_node>());
    rclcpp::shutdown();
    return 0;
}