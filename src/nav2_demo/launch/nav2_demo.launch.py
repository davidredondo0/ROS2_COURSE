import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    # --------------------
    # PATHS
    # --------------------
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # --------------------
    # ARGUMENTOS
    # --------------------
    declare_model = DeclareLaunchArgument(
        'model',
        default_value='waffle',
        description='TurtleBot3 model'
    )

    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value=LaunchConfiguration('model')
    )

    # --------------------
    # 1. GAZEBO
    # --------------------
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # --------------------
    # 2. NAV2 (espera 10s usando TimerAction)
    # --------------------
    nav2_launch = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'True',
                'slam': 'True',
                'autostart': 'True',
                'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
            }.items())]
    )

    # --------------------
    # 3. SLAM Toolbox (espera 10s más)
    # --------------------
    slam_launch = TimerAction(
        period=5.0,  # 20s desde inicio
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'True'
            }.items()
        )]
    )

    # --------------------
    # 4. RVIZ (espera 30s desde inicio)
    # --------------------
    rviz_launch = TimerAction(
        period=15.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'run', 'rviz2', 'rviz2',
                '-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'
            ],
            output='screen'
        )]
    )

    # --------------------
    # Launch Description
    # --------------------
    return LaunchDescription([
        declare_model,
        set_tb3_model,
        gazebo_sim,
        nav2_launch,
        slam_launch,
        rviz_launch
    ])
