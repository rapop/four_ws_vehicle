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

namespace wheeled_robot_tf_broadcaster
{

  typedef const boost::shared_ptr<gazebo::msgs::PosesStamped const> ConstPosesStampedPtr;

  class wheeled_robot_tf_broadcaster_node : public rclcpp::Node
  {
  public:
    wheeled_robot_tf_broadcaster_node();

  private:
    gazebo::transport::NodePtr _gazebo_node;
    gazebo::transport::SubscriberPtr _gazebo_sub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    const std::string _gazebo_poses_topic_name = "/gazebo/world/pose/info";
    const std::string _dynamic_joint_state_topic_name = "/dynamic_joint_states";
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr _dynamic_joint_state_sub;

    const std::string _front_right_wheel_steering_joint_angle_name = "front_right_wheel_steering_joint";
    const std::string _front_left_wheel_steering_joint_angle_name = "front_left_wheel_steering_joint";

    const std::string _front_right_wheel_joint_angle_name = "front_right_wheel_joint";
    const std::string _front_left_wheel_joint_angle_name = "front_left_wheel_joint";
    const std::string _back_right_wheel_joint_angle_name = "back_right_wheel_joint";
    const std::string _back_left_wheel_joint_angle_name = "back_left_wheel_joint";

    const std::string _position_interface_name = "position";

    const std::string _ground_truth_pose_topic_name = "/ground_truth";

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _ground_truth_pose_publisher;

    void _on_poses_callback(ConstPosesStampedPtr &msg);
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

} // namespace wheeled_robot_tf_broadcaster