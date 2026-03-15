import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import TimerAction, ExecuteProcess

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='maze_solver_bot' #<--- CHANGE ME
    world_file=os.path.join(get_package_share_directory(package_name), 'worlds', 'Maze.world')
    rviz_file=os.path.join(get_package_share_directory(package_name), 'config', 'R2_Navigation.rviz')
    slam_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml')
    ekf_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    jsc = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]),
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'true',
        }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation_launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'map_subscribe_transient_local': 'true'}.items()
    )
    

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument('world', default_value=world_file, description='World to load')


    # Include the Gazebo launch file, provided by the gazebo_ros package
    gz_sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items(),
             )
    

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            'config',
            'diff_drive_controller.yaml',
        ]
    )


    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-x', '0.0', '-y', '0.0', '-z', '1.5',
        ],
        output='screen'
    )

    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            robot_controllers,
            ],
        remappings=[
            ('/diff_drive_base_controller/cmd_vel', '/cmd_vel')  # clean remap, no relay needed
        ],
    )

    #Relay Cmd_vEL
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=[
            '/cmd_vel',
            '/diff_drive_base_controller/cmd_vel'
        ],
        output='screen'
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name='rviz2',
        arguments=['-d', rviz_file],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': True}
        ]
    )





    follow_cam = ExecuteProcess(
        cmd=[
            'gz', 'service',
            '-s', '/gui/follow',
            '--reqtype', 'gz.msgs.StringMsg',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', 'data: "my_bot"'
        ],
        output='screen'
    )




    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner



    spawn_robot_delayed = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )

    slip_detector = Node(
        package='maze_solver_bot',
        executable='slip_detection.py',
        output='screen'
    )



    # Launch them all!
    return LaunchDescription([
        rsp,
        world_arg,
        gz_sim,
        ros_gz_bridge,
        rviz2_node,
        jsc,
        slam_toolbox_launch,
        navigation,
        cmd_vel_relay,
        spawn_robot,

        # Chain: spawn → joint_state_broadcaster → diff_drive → follow_cam
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_base_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=diff_drive_base_controller_spawner,
                on_exit=[follow_cam],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=follow_cam,
                on_exit=[ekf_node]
            )
        ),
        # slip_detector,
    ])
