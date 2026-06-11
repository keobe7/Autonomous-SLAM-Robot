import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = get_package_share_directory('custom_robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    rviz_config_path = os.path.join(pkg_description, 'rviz', 'map_robotmodel_scan.rviz')
    xacro_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_description, 'worlds', 'final_test.sdf')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_toggle = LaunchConfiguration('rviz', default='true')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    gazebo_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    robot_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='independent_robot_spawner',
        arguments=[
            '-name', 'independent_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    delayed_robot_spawner = TimerAction(
    period=4.0,
    actions=[robot_spawner]
    )

    bridge_params = os.path.join(pkg_description, 'config', 'bridge_parameters.yaml')

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
        '--ros-args', '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    rviz_instance = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz_toggle),
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Toggle RViz Frame Editor Display'),
        robot_state_publisher,
        gazebo_backend,
        delayed_robot_spawner,
        ros_gz_bridge,
        rviz_instance
    ])
