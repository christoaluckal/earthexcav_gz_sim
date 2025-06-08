from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    parameter = os.path.join(get_package_share_directory('deltacan_to_gz'), 'config', 'params.yaml')

    converter_node = Node(
        package='deltacan_to_gz',
        executable='control_exe',
        name='control_node',
        output='screen',
        parameters=[parameter]
    )


    ld.add_action(converter_node)

    return ld