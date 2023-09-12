import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess

from launch_ros.actions import Node


def generate_launch_description():

    world = os.path.join(get_package_share_directory(
    'four_ws_vehicle_simulation'), 'models', 'world.sdf')

    robot = os.path.join(get_package_share_directory(
    'four_ws_vehicle_simulation'), 'models', 'wheeled_robot.urdf')

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['wheeled_robot.rviz']
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        arguments=[robot]
     )

    tf = Node(
      package = 'tf2_ros',
      executable = 'static_transform_publisher', 
      arguments=['0', '0', '0.4', '0', '0', '0', 'world', 'wheeled_robot']
      )

    tf2 = Node(
       package = 'tf2_ros',
       executable = 'static_transform_publisher', 
       arguments=['0', '0', '0', '0', '0', '0', 'wheeled_robot', 'chassis']
       )
    
    forward_position_controller = ExecuteProcess( 
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_position_controller'], output='screen'
        )

    forward_velocity_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller'], output='screen'
        )

    gazebo = ExecuteProcess(
            cmd=['gazebo','--verbose', world],
            output='screen')

    spawn_robot = ExecuteProcess(
        cmd=['gz', 'model', '--spawn-file', robot, '-m', 'robot_name'],
        output='screen'
    )

    spawn_robot_delay = TimerAction(
            period=3.0,
            actions=[spawn_robot])

    with open(robot, 'r') as infp:
      robot_desc = infp.read()

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                         'robot_description': robot_desc,
                        }],
            arguments=[robot])

    controller_manager_node = Node(
            package="controller_manager",
            executable="spawner",   
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            )
    
    controller = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
    'four_ws_vehicle_controller'), 'launch', 'four_ws_vehicle_controller.launch.py')
            ),
        )
    
    teleop_twist_keyboard = Node(
        package = "teleop_twist_keyboard",
        executable = "teleop_twist_keyboard",
        prefix = 'xterm -e',
        output='screen',
        remappings=[
            ("cmd_vel", "robot_command")
        ]
        )

    return LaunchDescription([
         robot_state_publisher,
         joint_state_publisher,
        gazebo,
        spawn_robot_delay,
         tf2,
          tf,
          controller_manager_node,
          controller,
          forward_position_controller,
          forward_velocity_controller,
       rviz,
       teleop_twist_keyboard,
    ])
