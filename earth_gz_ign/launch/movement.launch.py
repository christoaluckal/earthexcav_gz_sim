# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gz_args = LaunchConfiguration('gz_args', default='')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('earth_gz_ign'),
                 'urdf', 'robot.xacro']
            ),
        ]
    )
    bridge_config_path = PathJoinSubstitution([FindPackageShare('earth_gz_ign'), 'config', 'bridge.yaml'])
    robot_description = {'robot_description': robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('earth_gz_ign'),
            'params',
            'vc.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'cart', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    deltacan_to_gz = Node(
        package='deltacan_to_gz',
        executable='control_exe',
        output='screen',
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    joy_to_deltacan = Node(
        package='deltacan_to_gz',
        executable='joy_to_deltacan_exe',
        name='joy_to_deltacan',
        output='screen',
    )

    # swing_to_boom_controller = Node(
    #     package = 'controller_manager',
    #     executable = 'spawner',
    #     arguments = [
    #         'swing_to_boom_controller',
    #         '--param-file',
    #         robot_controllers
    #     ],
    # )
    # boom_to_arm_controller = Node(
    #     package = 'controller_manager',
    #     executable = 'spawner',
    #     arguments = [
    #         'boom_to_arm_controller',
    #         '--param-file',
    #         robot_controllers
    #     ],
    # )

    # arm_to_bucket_controller = Node(
    #     package = 'controller_manager',
    #     executable = 'spawner',
    #     arguments = [
    #         'arm_to_bucket_controller',
    #         '--param-file',
    #         robot_controllers
    #     ],
    # )

    manipulator_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'manipulator_controller',
            '--param-file',
            robot_controllers
        ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/demo/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                   '/demo/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
                   ],
        parameters=[{
            'config_file': bridge_config_path,  
        }],
        output='screen'
    )



    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [gz_args, ' -r -v 1 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                # on_exit=[swing_to_boom_controller, boom_to_arm_controller, arm_to_bucket_controller],
                on_exit=[manipulator_controller],
            )
        ),
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        deltacan_to_gz,
        joy,
        joy_to_deltacan,
    ])
