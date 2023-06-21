
import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Start the nodes required for the majority rule pattern."""
    robot_namespace = LaunchConfiguration('robot_namespace', default='robot_namespace_default')
    config_dir = LaunchConfiguration('config_dir', default='config_dir_default')
    log_level = LaunchConfiguration("log_level", default='debug')

    ld = LaunchDescription()
    ros2_pattern_node = launch_ros.actions.Node(
        package='ros2swarm',
        executable='sequential_execution_pattern',
        namespace=robot_namespace,
        output='screen',
        parameters=[
        PathJoinSubstitution([config_dir, 'sequential_execution_pattern.yaml']),
        PathJoinSubstitution([config_dir, 'movement_pattern', 'basic', 'random_walk.yaml']),
        PathJoinSubstitution([config_dir, 'voting_pattern', 'basic', 'voter_model_pattern.yaml'])
    ],
        arguments=['--ros-args', '--log-level', log_level]
    )
    ld.add_action(ros2_pattern_node)

    return ld
