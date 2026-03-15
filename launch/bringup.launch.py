import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name='maze_solver_bot'
    pkg_share = FindPackageShare('maze_solver_bot')


    map_file=os.path.join(get_package_share_directory(package_name), 'maps', 'RoboLvl_edited.yaml')
    rviz_file=os.path.join(get_package_share_directory(package_name), 'config', 'R2_Navigation.rviz')
    

    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'launch_sim.launch.py'])
        )
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'localization_launch.py'])
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true'
        }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map_subscribe_transient_local': 'true'
        }.items()
    )


    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name='rviz2',
        arguments=['-d', rviz_file],
        output='screen'
    )

    return LaunchDescription([
        launch_sim,
        rviz2_node,
        localization,
        navigation,
    ])
