"""Multi-robot exploration exam.

Starts Gazebo, one CMU-style autonomy stack per robot (namespaced robot_0 ...
robot_{n-1}), a shared octomap of the whole team, RViz, the team evaluator and,
optionally, the candidate's `explorer` package.

    ros2 launch exam_bringup exam.launch.py world_name:=indoor
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

# Seconds between robot spawns; spawning several entities at once is unreliable in Gazebo Classic.
SPAWN_INTERVAL = 2.0
# Extra wait after the last robot is spawned before the explorer starts.
EXPLORER_DELAY = 8.0


def generate_robots(context):
    n = int(LaunchConfiguration('n_robots').perform(context))
    world_name = LaunchConfiguration('world_name').perform(context)
    xs = yaml.safe_load(LaunchConfiguration('vehicleX_list').perform(context))
    ys = yaml.safe_load(LaunchConfiguration('vehicleY_list').perform(context))
    xs = xs if isinstance(xs, list) else [xs]
    ys = ys if isinstance(ys, list) else [ys]
    if len(xs) < n or len(ys) < n:
        raise RuntimeError(f'n_robots={n} but only {min(len(xs), len(ys))} start positions were given')

    system_launch = os.path.join(get_package_share_directory('vehicle_simulator'), 'launch', 'system.launch.py')
    actions = []
    for i in range(n):
        ns = f'robot_{i}'
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(system_launch),
            launch_arguments={
                'namespace': ns,
                'world_name': world_name,
                'vehicleX': str(xs[i]),
                'vehicleY': str(ys[i]),
                'vis_tools': 'false',
                'gazebo_gui': LaunchConfiguration('gazebo_gui').perform(context),
            }.items(),
        )
        actions.append(TimerAction(period=SPAWN_INTERVAL * i, actions=[GroupAction([PushRosNamespace(ns), include])]))

    # One octomap_server for the whole team, fed with every robot's sensor_scan.
    mapping_launch = os.path.join(get_package_share_directory('exam_bringup'), 'launch', f'mapping_{world_name}.launch')
    if not os.path.exists(mapping_launch):
        raise RuntimeError(f'No mapping config for world "{world_name}" (expected {mapping_launch})')
    actions.append(Node(
        package='exam_bringup',
        executable='scan_merger',
        name='scan_merger',
        parameters=[{'n_robots': n}],
    ))
    actions.append(IncludeLaunchDescription(FrontendLaunchDescriptionSource(mapping_launch)))

    actions.append(Node(
        package='exam_bringup',
        executable='evaluator',
        name='exam_evaluator',
        output='screen',
        parameters=[{
            'n_robots': n,
            'time_limit': float(LaunchConfiguration('time_limit').perform(context)),
            'log_dir': LaunchConfiguration('log_dir').perform(context),
        }],
    ))

    if LaunchConfiguration('start_explorer').perform(context) == 'true':
        explorer_launch = os.path.join(get_package_share_directory('explorer'), 'launch', 'explorer.launch.py')
        actions.append(TimerAction(
            period=SPAWN_INTERVAL * n + EXPLORER_DELAY,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(explorer_launch),
                launch_arguments={'n_robots': str(n)}.items(),
            )],
        ))

    return actions


def declare_world(context):
    world_name = LaunchConfiguration('world_name').perform(context)
    world = os.path.join(get_package_share_directory('vehicle_simulator'), 'world', world_name + '.world')
    gazebo_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'world': world, 'gui': LaunchConfiguration('gazebo_gui'), 'verbose': 'false'}.items(),
    )]


def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('vehicle_simulator'), 'rviz', 'vehicle_simulator.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='indoor',
                              description='indoor | garage | tunnel | forest'),
        DeclareLaunchArgument('n_robots', default_value='3'),
        DeclareLaunchArgument('vehicleX_list', default_value='[0.0, 23.01, 32.06]',
                              description='Start x of each robot (map frame)'),
        DeclareLaunchArgument('vehicleY_list', default_value='[0.0, 11.62, -6.57]',
                              description='Start y of each robot (map frame)'),
        DeclareLaunchArgument('time_limit', default_value='300.0',
                              description='Seconds from the first way_point until evaluation stops'),
        DeclareLaunchArgument('log_dir', default_value='exam_results'),
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('start_explorer', default_value='true',
                              description='Also launch explorer/launch/explorer.launch.py'),

        OpaqueFunction(function=declare_world),
        OpaqueFunction(function=generate_robots),
        TimerAction(period=8.0, actions=[Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config])]),
    ])
