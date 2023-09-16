#include <odom_visualizer/odom_visualizer_node.h>

#include <string>
#include <memory>
#include <stdexcept>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>


using namespace std::chrono_literals;

namespace odom_visualizer
{
    odom_visualizer_node::odom_visualizer_node()
        : Node("odom_visualizer_node")
    {
         this->declare_parameter("markers_pos_sub_topic_name", "markers_pos");
         this->declare_parameter("arrow_markers_pub_topic_name", "arrow_markers");
         this->declare_parameter("path_markers_pub_topic_name", "path_markers");

         this->declare_parameter("a", 1.0);
         this->declare_parameter("r", 1.0);
         this->declare_parameter("g", 1.0);
         this->declare_parameter("b", 1.0);

        const std::string markers_pos_sub_topic_name = this->get_parameter("markers_pos_sub_topic_name").as_string();
        const std::string arrow_markers_pub_topic_name = this->get_parameter("arrow_markers_pub_topic_name").as_string();
        const std::string path_markers_pub_topic_name = this->get_parameter("path_markers_pub_topic_name").as_string();

         _a = this->get_parameter("a").as_double();
         _r = this->get_parameter("r").as_double();
         _g = this->get_parameter("g").as_double();
         _b = this->get_parameter("b").as_double();

        const auto qos = rclcpp::SystemDefaultsQoS();

        _marker_pos_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            markers_pos_sub_topic_name, qos,
            std::bind(&odom_visualizer_node::_marker_pos_sub_cb, this, std::placeholders::_1));
    
        _arrow_marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>
        (arrow_markers_pub_topic_name, qos);

        _path_publisher = this->create_publisher<nav_msgs::msg::Path>(path_markers_pub_topic_name, qos);

        _path_msg.header.frame_id = "world";
        _path_msg.header.stamp = this->get_clock()->now();
    }


    void odom_visualizer_node::_marker_pos_sub_cb(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        visualization_msgs::msg::Marker arrow_marker;

        arrow_marker.header.frame_id = "world";
        arrow_marker.header.stamp = this->get_clock()->now();
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;

        geometry_msgs::msg::Pose pose;

        pose.position.x = msg->pose.position.x;  
        pose.position.y = msg->pose.position.y;

        pose.orientation.x = msg->pose.orientation.x;
        pose.orientation.y = msg->pose.orientation.y;
        pose.orientation.z = msg->pose.orientation.z;
        pose.orientation.w = msg->pose.orientation.w;


        arrow_marker.pose = pose;
        arrow_marker.id = _arrow_id_counter++;
        arrow_marker.scale.x = 0.5;
        arrow_marker.scale.y = 0.1;
        arrow_marker.scale.z = 0.1;

        std_msgs::msg::ColorRGBA colour;
        // colour.a = 0.5; //transparency
        // colour.r = 0.6;
        // colour.g = 0.76;
        // colour.b = 0.95;
        colour.a = _a; //transparency
        colour.r = _r;
        colour.g = _g;
        colour.b = _b;

        arrow_marker.color = colour;
        // arrow_marker.lifetime.sec = 10.0;

        if (abs(_last_published_arrow_id_counter - _arrow_id_counter) > _published_arrow_step)
        {
            _arrow_marker_publisher->publish(arrow_marker);
            _last_published_arrow_id_counter = _arrow_id_counter;
        }

        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "world";
        p.header.stamp = this->get_clock()->now();
        p.pose.position.x = msg->pose.position.x;
        p.pose.position.y = msg->pose.position.y;

        _path_msg.poses.push_back(p);

        _path_publisher->publish(_path_msg);
    }

} // namespace odom_visualizer

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<odom_visualizer::odom_visualizer_node>());
    rclcpp::shutdown();
    return 0;
}
