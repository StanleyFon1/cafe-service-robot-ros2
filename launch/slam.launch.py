import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('serv_robot')
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg, 'config', 'slam.rviz')

    return LaunchDescription([
        # Bring up Gazebo + robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'gazebo.launch.py')
            ),
        ),

        # SLAM Toolbox in async mapping mode
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params],
            output='screen',
        ),

        # RViz for watching the map build
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
