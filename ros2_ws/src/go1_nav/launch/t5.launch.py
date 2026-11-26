import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    pkg_nav = get_package_share_directory('go1_nav')
    pkg_slam = get_package_share_directory('slam_toolbox')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_slam, 'config', 'mapper_params_online_async.yaml'),
            {'use_sim_time': True, 'base_frame': 'trunk'},
        ],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
        }.items(),
    )

    return LaunchDescription([
        slam_node,
        GroupAction([
            SetRemap(src='/goal_pose', dst='/nav2_goal_pose'),
            nav2_launch,
        ]),
        Node(
            package='go1_nav',
            executable='goal_relay',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
