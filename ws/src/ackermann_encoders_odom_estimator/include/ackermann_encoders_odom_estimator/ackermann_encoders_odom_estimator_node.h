#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/string.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace ackermann_encoders_odom_estimator
{
  class ackermann_encoders_odom_estimator_node : public rclcpp::Node
  {
  public:
    ackermann_encoders_odom_estimator_node();

  private:
    const std::string _dynamic_joint_state_topic_name = "/dynamic_joint_states";
    const std::string _estimated_pose_topic_name = "/ackermann_encoders_odom_estimation";
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr _dynamic_joint_state_sub;

    const std::string _front_right_wheel_steering_joint_angle_name = "front_right_wheel_steering_joint";
    const std::string _front_left_wheel_steering_joint_angle_name = "front_left_wheel_steering_joint";

    const std::string _front_right_wheel_joint_angle_name = "front_right_wheel_joint";
    const std::string _front_left_wheel_joint_angle_name = "front_left_wheel_joint";
    const std::string _back_right_wheel_joint_angle_name = "back_right_wheel_joint";
    const std::string _back_left_wheel_joint_angle_name = "back_left_wheel_joint";

    const std::string _position_interface_name = "position";

    const double _wheel_radius = 0.4;
    double _back_left_wheel_traveled_distance = 0;
    double _back_right_wheel_traveled_distance = 0;
    double _last_back_left_wheel_joint_angle = 0;
    double _last_back_right_wheel_joint_angle = 0;
    double _last_yaw_angle = 0;
    double _wheel_base = 1;
    double _last_estimated_x = 0;
    double _last_estimated_y = 0;
    double _last_estimated_yaw = 0;


    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _estimated_pose_publisher;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    void _dynamic_joint_state_sub_cb(
        const control_msgs::msg::DynamicJointState::SharedPtr msg);
    // This is necessary because the indeces are not fixed but the names are.
    double _get_dynamic_state_value(const std::string &joint_name,
           const std::string &interface_name, const control_msgs::msg::DynamicJointState::SharedPtr msg);
    size_t _get_interface_index(
        const std::string &interface_name,
        size_t joint_index, 
        const control_msgs::msg::DynamicJointState::SharedPtr msg);
    size_t _get_joint_index(const std::string &joint_name, 
            const control_msgs::msg::DynamicJointState::SharedPtr msg);
  };

} // namespace ackermann_encoders_odom_estimator