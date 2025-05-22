from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()
    parameter = os.path.join(get_package_share_directory('empty_map'), 'config', 'params.yaml')


    converter_node = Node(
        package='empty_map',
        executable='map_exe',
        name='map_node',
        output='screen',
        parameters=[parameter]
    )


    ld.add_action(converter_node)

    return ld