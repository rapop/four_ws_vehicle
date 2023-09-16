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
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>

namespace odom_visualizer
{
  class odom_visualizer_node : public rclcpp::Node
  {
  public:
    odom_visualizer_node();

  private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _marker_pos_sub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _arrow_marker_publisher;

    nav_msgs::msg::Path _path_msg;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_publisher;

    int _arrow_id_counter = 0;
    int _path_id_counter = 0;
    int _last_published_arrow_id_counter = 0;
    int _published_arrow_step = 2000;
    double _a;
    double _r;
    double _g;
    double _b;
    
    void _marker_pos_sub_cb(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  };

} // namespace odom_visualizer