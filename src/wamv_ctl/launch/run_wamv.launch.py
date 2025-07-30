import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='wamv_ctl',
            executable='inverse_kinematics',
            name='inverse_kinematics'),
        launch_ros.actions.Node(
            package='wamv_ctl',
            executable='station_keeping',
            name='station_keeping'),
    ])
