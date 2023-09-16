#include <wheeled_robot_tf_broadcaster/wheeled_robot_tf_broadcaster_node.h>

#include <string>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <gazebo/msgs/msgs.hh>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

namespace wheeled_robot_tf_broadcaster
{
    wheeled_robot_tf_broadcaster_node::wheeled_robot_tf_broadcaster_node()
        : Node("wheeled_robot_tf_broadcaster_node")
    {
        _gazebo_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
        _gazebo_node->Init();

        _gazebo_sub = _gazebo_node->Subscribe(_gazebo_poses_topic_name,
                                              &wheeled_robot_tf_broadcaster_node::_on_poses_callback, this);

        if (!_gazebo_sub)
        {
            throw std::runtime_error("Failed to subscribe to gazebo topic : " + _gazebo_poses_topic_name);
        }

        _tf_broadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        std::cout << "Listening for poses on topic: " << _gazebo_sub->GetTopic() << std::endl;

        _ground_truth_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>
        (_ground_truth_pose_topic_name, rclcpp::SystemDefaultsQoS());

        _dynamic_joint_state_sub = this->create_subscription<control_msgs::msg::DynamicJointState>(
            _dynamic_joint_state_topic_name, rclcpp::SystemDefaultsQoS(),
            std::bind(&wheeled_robot_tf_broadcaster_node::_dynamic_joint_state_sub_cb, this, std::placeholders::_1));
    }

    size_t wheeled_robot_tf_broadcaster_node::_get_joint_index(const std::string &joint_name,
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

    size_t wheeled_robot_tf_broadcaster_node::_get_interface_index(
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

    double wheeled_robot_tf_broadcaster_node::_get_dynamic_state_value(const std::string &joint_name,
                                                                       const std::string &interface_name,
                                                                       const control_msgs::msg::DynamicJointState::SharedPtr msg)
    {
        const size_t joint_index = _get_joint_index(joint_name, msg);
        const size_t interface_index = _get_interface_index(interface_name, joint_index, msg);

        return msg->interface_values[joint_index].values[interface_index];
    }

    void wheeled_robot_tf_broadcaster_node::_dynamic_joint_state_sub_cb(
        const control_msgs::msg::DynamicJointState::SharedPtr msg)
    {
//         // send wheels steering transforms
//         const double front_right_wheel_steering_joint_angle =
//             _get_dynamic_state_value(_front_right_wheel_steering_joint_angle_name,
//                                      _position_interface_name, msg);

//         const double front_left_wheel_steering_joint_angle =
//             _get_dynamic_state_value(_front_left_wheel_steering_joint_angle_name,
//                                      _position_interface_name, msg);

//         geometry_msgs::msg::TransformStamped front_right_wheel_steering_joint_angle_t;

//         front_right_wheel_steering_joint_angle_t.header.stamp = this->get_clock()->now();
//         front_right_wheel_steering_joint_angle_t.header.frame_id = "chassis";
//         front_right_wheel_steering_joint_angle_t.child_frame_id = "front_right_wheel_steering";

//         tf2::Quaternion front_right_wheel_steering_joint_angle_q;
//         front_right_wheel_steering_joint_angle_q.setRPY(0, 1.5, 0);

//         front_right_wheel_steering_joint_angle_t.transform.rotation.x = front_right_wheel_steering_joint_angle_q.getX();
//         front_right_wheel_steering_joint_angle_t.transform.rotation.y = front_right_wheel_steering_joint_angle_q.getY();
//         front_right_wheel_steering_joint_angle_t.transform.rotation.z = front_right_wheel_steering_joint_angle_q.getZ();
//         front_right_wheel_steering_joint_angle_t.transform.rotation.w = front_right_wheel_steering_joint_angle_q.getW();

// std::cout << " send front_right_wheel_steering_joint_angle: " << front_right_wheel_steering_joint_angle << std::endl;
//         //  _tf_broadcaster->sendTransform(front_right_wheel_steering_joint_angle_t);

//         geometry_msgs::msg::TransformStamped front_left_wheel_steering_joint_angle_t;

//         front_left_wheel_steering_joint_angle_t.header.stamp = this->get_clock()->now();
//         front_left_wheel_steering_joint_angle_t.header.frame_id = "front_left_wheel_steering";
//         front_left_wheel_steering_joint_angle_t.child_frame_id = "front_left_wheel";

//         tf2::Quaternion front_left_wheel_steering_joint_angle_q;
//         front_left_wheel_steering_joint_angle_q.setRPY(0, 0, front_left_wheel_steering_joint_angle);

//         std::cout << " send front_left_wheel_steering_joint_angle: " << front_left_wheel_steering_joint_angle << std::endl;

//         front_left_wheel_steering_joint_angle_t.transform.rotation.x = front_left_wheel_steering_joint_angle_q.getX();
//         front_left_wheel_steering_joint_angle_t.transform.rotation.y = front_left_wheel_steering_joint_angle_q.getY();
//         front_left_wheel_steering_joint_angle_t.transform.rotation.z = front_left_wheel_steering_joint_angle_q.getZ();
//         front_left_wheel_steering_joint_angle_t.transform.rotation.w = front_left_wheel_steering_joint_angle_q.getW();

//         // _tf_broadcaster->sendTransform(front_left_wheel_steering_joint_angle_t);


//         // send wheels rotations transforms
//         const double front_right_wheel_joint_angle =
//             _get_dynamic_state_value(_front_right_wheel_joint_angle_name,
//                                      _position_interface_name, msg);
                            
//         // const double front_right_wheel_joint_angle =
//         //     _get_dynamic_state_value(_front_right_wheel_joint_angle_name,
//         //                              _position_interface_name, msg);

//         // const double front_right_wheel_joint_angle =
//         //     _get_dynamic_state_value(_front_right_wheel_joint_angle_name,
//         //                              _position_interface_name, msg);

//         // const double front_right_wheel_joint_angle =
//         //     _get_dynamic_state_value(_front_right_wheel_joint_angle_name,
//         //                              _position_interface_name, msg);


//         geometry_msgs::msg::TransformStamped front_right_wheel_joint_angle_t;

//         front_right_wheel_joint_angle_t.header.stamp = this->get_clock()->now();
//         front_right_wheel_joint_angle_t.header.frame_id = "front_right_wheel_steering";
//         front_right_wheel_joint_angle_t.child_frame_id = "front_right_wheel";

//         tf2::Quaternion front_right_wheel_joint_angle_q;
//         front_right_wheel_joint_angle_q.setRPY(0, front_right_wheel_joint_angle, 1.5);

//         front_right_wheel_joint_angle_t.transform.rotation.x = front_right_wheel_joint_angle_q.getX();
//         front_right_wheel_joint_angle_t.transform.rotation.y = front_right_wheel_joint_angle_q.getY();
//         front_right_wheel_joint_angle_t.transform.rotation.z = front_right_wheel_joint_angle_q.getZ();
//         front_right_wheel_joint_angle_t.transform.rotation.w = front_right_wheel_joint_angle_q.getW();

//         // std::cout << " send front_right_wheel_steering_joint_angle: " << front_right_wheel_steering_joint_angle << std::endl;
//         _tf_broadcaster->sendTransform(front_right_wheel_joint_angle_t);

    }

    void wheeled_robot_tf_broadcaster_node::_on_poses_callback(ConstPosesStampedPtr &msg)
    {
        const size_t nb_of_objects = msg->pose_size();
        for (size_t i = 0; i < nb_of_objects; i++)
        {
            const gazebo::msgs::Pose object_pose = msg->pose(i);
            if (object_pose.name() == "my_robot_name")
            {
                geometry_msgs::msg::TransformStamped t;

                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = "world";
                t.child_frame_id = "wheeled_robot";

                t.transform.translation.x = object_pose.position().x();
                t.transform.translation.y = object_pose.position().y();
                t.transform.translation.z = object_pose.position().z();

                t.transform.rotation.x = object_pose.orientation().x();
                t.transform.rotation.y = object_pose.orientation().y();
                t.transform.rotation.z = object_pose.orientation().z();
                t.transform.rotation.w = object_pose.orientation().w();

                _tf_broadcaster->sendTransform(t);

                // RCLCPP_INFO(this->get_logger(), "Sending transform for x: %f", object_pose.position().x());

                geometry_msgs::msg::PoseStamped ground_truth_pose;

                ground_truth_pose.header.stamp = this->get_clock()->now();
                ground_truth_pose.header.frame_id = "ground_truth_pose";

                ground_truth_pose.pose.position.x = object_pose.position().x();
                ground_truth_pose.pose.position.y = object_pose.position().y();

                ground_truth_pose.pose.orientation.x = object_pose.orientation().x();
                ground_truth_pose.pose.orientation.y = object_pose.orientation().y();
                ground_truth_pose.pose.orientation.z = object_pose.orientation().z();
                ground_truth_pose.pose.orientation.w = object_pose.orientation().w();

                _ground_truth_pose_publisher->publish(ground_truth_pose);

            }
        }
    }

} // namespace wheeled_robot_tf_broadcaster

int main(int argc, char *argv[])
{
    gazebo::transport::init();
    gazebo::transport::run();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wheeled_robot_tf_broadcaster::wheeled_robot_tf_broadcaster_node>());
    rclcpp::shutdown();
    return 0;
}
