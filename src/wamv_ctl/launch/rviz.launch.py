import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os.path

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('wamv_ctl'), 'config', 'path_static_tf.rviz')],) 
    ])
