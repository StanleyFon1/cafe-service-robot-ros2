import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('serv_robot')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    urdf_file = os.path.join(pkg, 'urdf', 'serv_robot.urdf')
    world_file = os.path.join(pkg, 'worlds', 'cafe.world')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH',
            os.path.dirname(pkg) + ':' +
            os.environ.get('GAZEBO_MODEL_PATH', '')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file, 'gui': 'false'}.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}],
        ),
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_entity',
                    arguments=[
                        '-entity', 'serv_robot',
                        '-topic', '/robot_description',
                        '-x', '1.75', '-y', '-11.0', '-z', '0.05',
                        '-Y', '0',
                    ],
                    output='screen',
                ),
            ],
        ),
    ])
