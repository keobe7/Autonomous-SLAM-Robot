import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = get_package_share_directory('custom_robot_description')
    pkg_ros_gz_sim  = get_package_share_directory('ros_gz_sim')
    
    xacro_file    = os.path.join(pkg_description, 'urdf',   'robot.urdf.xacro')
    world_file    = os.path.join(pkg_description, 'worlds', 'final_test.sdf')
    bridge_params = os.path.join(pkg_description, 'config', 'bridge_parameters.yaml')
    nav2_params   = os.path.join(pkg_description, 'config', 'nav2_localization.yaml')
    rviz_config   = os.path.join(pkg_description, 'rviz',   'map_robotmodel_scan.rviz')
    map_yaml_file = os.path.join(pkg_description, 'maps', 'my_map2.yaml')

    rviz_toggle = LaunchConfiguration('rviz', default='true')

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
        'use_sim_time': True,
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str
            )
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

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    nav2_params_list = [nav2_params, {'use_sim_time': True}]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file, 'use_sim_time': True}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=nav2_params_list
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=nav2_params_list,
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=nav2_params_list,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=nav2_params_list,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=nav2_params_list,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=nav2_params_list,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=nav2_params_list,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=nav2_params_list,
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    nav2_nodes = [
        map_server,
        amcl,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager_navigation,
    ]

    delayed_nav2 = TimerAction(
        period=8.0,
        actions=nav2_nodes
    )

    rviz_instance = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz_toggle),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_rviz,
        robot_state_publisher,
        gazebo_backend,
        robot_spawner,
        ros_gz_bridge,
        delayed_nav2,
        rviz_instance,
    ])
