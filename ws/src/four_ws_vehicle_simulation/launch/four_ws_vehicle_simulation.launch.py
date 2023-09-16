import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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
        cmd=['gz', 'model', '--spawn-file', robot, '-m', 'my_robot_name'],
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
                         'use_sim_time': use_sim_time,
                        }],
            arguments=[robot])
    

    # controller_manager_node = Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    #         )
    
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace="wheeled_robot",
        arguments=[
            "joint_state_broadcaster",
            # "--controller-manager-timeout",
            # "300",
            "--controller-manager",
            "/controller_manager",
        ],
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
    
    wheeled_robot_tf_broadcaster_node = Node(
       package='wheeled_robot_tf_broadcaster',
       executable='wheeled_robot_tf_broadcaster')
    
    ackermann_encoders_odom_estimator_node = Node(
       package = 'ackermann_encoders_odom_estimator',
       executable='ackermann_encoders_odom_estimator',
    )

    ackermann_encoders_odom_estimator_marker_publisher_node = Node(
       package = 'odom_visualizer',
       executable='odom_visualizer',
       name='ackermann_encoders_odom_estimator_marker_publisher_node',
    parameters=[
                {"markers_pos_sub_topic_name": "ackermann_encoders_odom_estimation"},
                {"arrow_markers_pub_topic_name": "ackermann_encoders_odom_estimation_markers"},
                {"path_markers_pub_topic_name": "ackermann_encoders_odom_estimation_path_markers"},
                {"a": 0.5},
                {"r": 0.6},
                {"g": 0.76},
                {"b": 0.95},
            ]
    )

    ground_truth_marker_publisher_node = Node(
       package = 'odom_visualizer',
       executable='odom_visualizer',
       name='ground_truth_marker_publisher_node',
    parameters=[
                {"markers_pos_sub_topic_name": "ground_truth"},
                {"arrow_markers_pub_topic_name": "ground_truth_markers"},
                {"path_markers_pub_topic_name": "ground_truth_path_markers"},
                {"a": 1.0},
                {"r": 0.1},
                {"g": 1.0},
                {"b": 0.0},
            ]
    )
    
    return LaunchDescription([
         robot_state_publisher,
         joint_state_publisher,
        gazebo,
        spawn_robot_delay,
         tf2,
          tf,
        joint_state_broadcaster_node,
          controller,
          forward_position_controller,
          forward_velocity_controller,
       rviz,
       teleop_twist_keyboard,
       wheeled_robot_tf_broadcaster_node,
       ackermann_encoders_odom_estimator_node,
       ackermann_encoders_odom_estimator_marker_publisher_node,
        ground_truth_marker_publisher_node,
    ])
