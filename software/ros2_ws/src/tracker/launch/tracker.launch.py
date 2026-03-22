from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('tracker')
    params = PathJoinSubstitution([pkg, 'config', 'params.yaml'])

    radar = Node(
        package='tracker',
        executable='radar_driver',
        name='radar_driver',
        parameters=[params],
        output='screen',
    )

    camera = Node(
        package='tracker',
        executable='camera_tracker',
        name='camera_tracker',
        parameters=[params],
        output='screen',
    )

    ekf = Node(
        package='tracker',
        executable='ekf_controller',
        name='ekf_controller',
        parameters=[params],
        output='screen',
    )

    gimbal = Node(
        package='tracker',
        executable='gimbal_controller',
        name='gimbal_controller',
        parameters=[params],
        output='screen',
    )

    return LaunchDescription([radar, camera, ekf, gimbal])
