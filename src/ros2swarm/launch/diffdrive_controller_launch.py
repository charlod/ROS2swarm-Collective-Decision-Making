from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',
            executable='diff_drive_controller',
            namespace='robot_namespace_0',
            output='screen'
        ),
    ])