
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wamv_ctl_cpp',
            executable='offboard_control_actuator_test',
            name='offboard_control_actuator_test',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'motor_left': 0.6},
                {'motor_right': 0.6},
                {'servo_left': 0.5},
                {'servo_right': 0.5},
            ]
        )
    ])

