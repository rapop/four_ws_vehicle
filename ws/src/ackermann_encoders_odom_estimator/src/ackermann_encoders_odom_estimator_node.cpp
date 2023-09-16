#include <ackermann_encoders_odom_estimator/ackermann_encoders_odom_estimator_node.h>

#include <string>
#include <memory>
#include <stdexcept>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

constexpr double EPSILON_STEERING_JOINT_ANGLE = 0.01;

namespace ackermann_encoders_odom_estimator
{
    ackermann_encoders_odom_estimator_node::ackermann_encoders_odom_estimator_node()
        : Node("ackermann_encoders_odom_estimator_node")
    {
        _dynamic_joint_state_sub = this->create_subscription<control_msgs::msg::DynamicJointState>(
            _dynamic_joint_state_topic_name, rclcpp::SystemDefaultsQoS(),
            std::bind(&ackermann_encoders_odom_estimator_node::_dynamic_joint_state_sub_cb, this, std::placeholders::_1));
    
        _estimated_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>
        (_estimated_pose_topic_name, rclcpp::SystemDefaultsQoS());

        _tf_broadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    size_t ackermann_encoders_odom_estimator_node::_get_joint_index(const std::string &joint_name,
                                                               const control_msgs::msg::DynamicJointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->joint_names.size(); ++i)
        {
            if (msg->joint_names[i] == joint_name)
            {
                return i;
            }
        }

        throw std::out_of_range(joint_name + " is not a joint name in joint_names vector");
    }

    size_t ackermann_encoders_odom_estimator_node::_get_interface_index(
        const std::string &interface_name,
        size_t joint_index,
        const control_msgs::msg::DynamicJointState::SharedPtr msg)
    {
        const std::vector<std::string> &interface_names =
            msg->interface_values[joint_index].interface_names;
        for (size_t i = 0; i < interface_names.size(); ++i)
        {
            if (interface_names[i] == interface_name)
            {
                return i;
            }
        }

        throw std::out_of_range(interface_name + " is not an interface name in interface_names vector");
    }

    double ackermann_encoders_odom_estimator_node::_get_dynamic_state_value(const std::string &joint_name,
                                                                       const std::string &interface_name,
                                                                       const control_msgs::msg::DynamicJointState::SharedPtr msg)
    {
        const size_t joint_index = _get_joint_index(joint_name, msg);
        const size_t interface_index = _get_interface_index(interface_name, joint_index, msg);

        return msg->interface_values[joint_index].values[interface_index];
    }

    void ackermann_encoders_odom_estimator_node::_dynamic_joint_state_sub_cb(
        const control_msgs::msg::DynamicJointState::SharedPtr msg)
    {
        
        const double front_right_wheel_steering_joint_angle =
            _get_dynamic_state_value(_front_right_wheel_steering_joint_angle_name,
                                     _position_interface_name, msg);

        const double front_left_wheel_steering_joint_angle =
            _get_dynamic_state_value(_front_left_wheel_steering_joint_angle_name,
                                     _position_interface_name, msg);


        const double front_right_wheel_joint_angle =
            _get_dynamic_state_value(_front_right_wheel_joint_angle_name,
                                     _position_interface_name, msg);
                            
        const double front_left_wheel_joint_angle =
            _get_dynamic_state_value(_front_left_wheel_joint_angle_name,
                                     _position_interface_name, msg);

        const double back_right_wheel_joint_angle =
            _get_dynamic_state_value(_back_right_wheel_joint_angle_name,
                                     _position_interface_name, msg);

        const double back_left_wheel_joint_angle =
            _get_dynamic_state_value(_back_left_wheel_joint_angle_name,
                                     _position_interface_name, msg);


        geometry_msgs::msg::PoseStamped estimated_pose;

        const double current_back_left_wheel_traveled_distance = 
        _wheel_radius*(back_left_wheel_joint_angle - _last_back_left_wheel_joint_angle);

        _back_left_wheel_traveled_distance += current_back_left_wheel_traveled_distance;

        _last_back_left_wheel_joint_angle = back_left_wheel_joint_angle;


        const double current_back_right_wheel_traveled_distance =
        _wheel_radius*(back_right_wheel_joint_angle - _last_back_right_wheel_joint_angle);

        _back_right_wheel_traveled_distance += current_back_right_wheel_traveled_distance;


        _last_back_right_wheel_joint_angle = back_right_wheel_joint_angle;


        double turning_radius = 0;
        // same angle assumed for right and left
        if (abs(front_right_wheel_steering_joint_angle) > EPSILON_STEERING_JOINT_ANGLE)
        {
            turning_radius = _wheel_base/
            tan(front_right_wheel_steering_joint_angle); 
        } 
        
        const double current_average_traveled_distance = 
        (current_back_right_wheel_traveled_distance + current_back_left_wheel_traveled_distance)/2.0;

        double delta_yaw = 0;
        if (abs(turning_radius) > 0)
        {
            delta_yaw = current_average_traveled_distance/turning_radius;
        }
        
        _last_estimated_x = _last_estimated_x + 
        current_average_traveled_distance*cos(_last_estimated_yaw + delta_yaw/2.0);

        _last_estimated_y = _last_estimated_y + 
        current_average_traveled_distance*sin(_last_estimated_yaw + delta_yaw/2.0);

        _last_estimated_yaw = _last_estimated_yaw + delta_yaw;

        if (_last_estimated_yaw >= 2*M_PI) 
        {
            _last_estimated_yaw = 0;
        }

        estimated_pose.header.stamp = this->get_clock()->now();
        estimated_pose.header.frame_id = "wheeled_robot_encoders_odom_estimate";

        estimated_pose.pose.position.x = _last_estimated_x;
        estimated_pose.pose.position.y = _last_estimated_y;

        tf2::Quaternion q;
        q.setRPY(0, 0, _last_estimated_yaw);

        estimated_pose.pose.orientation.x = q.x();
        estimated_pose.pose.orientation.y = q.y();
        estimated_pose.pose.orientation.z = q.z();
        estimated_pose.pose.orientation.w = q.w();

        _estimated_pose_publisher->publish(estimated_pose);
    }

} // namespace ackermann_encoders_odom_estimator

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ackermann_encoders_odom_estimator::ackermann_encoders_odom_estimator_node>());
    rclcpp::shutdown();
    return 0;
}
