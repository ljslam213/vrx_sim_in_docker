import launch
from launch.actions import IncludeLaunchDescription
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os.path

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='wamv_ctl',
            executable='inverse_kinematics',
            name='inverse_kinematics',
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),
        launch_ros.actions.Node(
            package='wamv_ctl',
            executable='path_follow_adpLOS',
            name='path_follow_adpLOS',
            arguments=['--ros-args', '--log-level', 'ERROR'],
        ),
        launch_ros.actions.Node(
            package='wamv_ctl',
            executable='eight_path',
            name='eight_path',
            arguments=['--ros-args', '--log-level', 'ERROR'],
        ),
        launch.actions.IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('vrx_gz'), 'launch', 'competition.launch.py']),
            launch_arguments=[
                ('world','sydney_regatta'),
            ],
        ),
        launch.actions.IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('wamv_ctl'), 'launch', 'rviz.launch.py'])
        ),
    ])
