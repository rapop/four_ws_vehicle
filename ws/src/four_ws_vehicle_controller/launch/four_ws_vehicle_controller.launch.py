from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():

    four_ws_vehicle_controller_node = Node(
        package='four_ws_vehicle_controller',
        executable='four_ws_vehicle_controller',
        output="both",
     )

    return LaunchDescription([
        four_ws_vehicle_controller_node
    ])
