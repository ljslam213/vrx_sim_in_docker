
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wamv_ctl_cpp',
            executable='uv_control',
            name='uv_control',
            output='screen',
            emulate_tty=True,
            parameters=[PathJoinSubstitution([
                FindPackageShare('wamv_ctl_cpp'), 'config', 'ctl_params.yaml'])
            ]
        )
    ])

