from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    parameter = os.path.join(get_package_share_directory('urdf_visualize'), 'config', 'params.yaml')

    converter_node = Node(
        package='urdf_visualize',
        executable='base2',
        name='base3',
        output='screen',
        parameters=[parameter]
    )


    ld.add_action(converter_node)

    return ld