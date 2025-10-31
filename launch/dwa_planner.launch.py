from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    pkg_share = get_package_share_directory('custom_dwa_planner')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'dwa_view.rviz')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py'))
    )
    dwa_node = Node(
        package='custom_dwa_planner',
        executable='dwa_planner_node',
        name='dwa_planner_node',
        output='screen',
        parameters=[params_file]
    )
    delayed_dwa = TimerAction(
        period=5.0,
        actions=[dwa_node]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz_node])
    return LaunchDescription([gazebo_launch, delayed_dwa, delayed_rviz])