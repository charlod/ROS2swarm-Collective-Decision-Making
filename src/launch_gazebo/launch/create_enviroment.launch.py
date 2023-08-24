from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import sys
import os


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='robot1',
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),


]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')

    # Paths
    ignition_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ignition.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_spawn.launch.py'])
    launch_bringup_dir = os.path.join(get_package_share_directory('ros2swarm'))

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    # for arg in sys.argv:
    #     if arg.startswith("pattern:="):  # The pattern executed by the robots
    #         pattern = arg.split(":=")[1]

    print("---------------------------------------")
    print("pattern          |", LaunchConfiguration('pattern'))
    print("---------------------------------------")


    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))]
    )

    launch_pattern_dir = os.path.join(get_package_share_directory('ros2swarm'), 'launch', 'pattern')

    # find out exact path of the pattern launch file
    pattern_launch_file_name = 'random_walk_pattern' + '.launch.py'
    for root, dirs, files in os.walk(launch_pattern_dir):
        for name in files:
            if name == pattern_launch_file_name:
                pattern_path = os.path.abspath(os.path.join(root, name))

    # add patterns
    launch_patterns = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_bringup_dir, '/' + 'bringup_patterns.launch.py']),
        launch_arguments={'robot': 'tb4',
                            'robot_namespace': LaunchConfiguration('namespace'),
                            'pattern': pattern_path
                        #   'params_file': os.path.join(
                                    # get_package_share_directory('ros2swarm'), 'param', 'nav2_params_' + robot_type + '_namespaced.yaml')
                        }.items()

    )
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
    ld.add_action(launch_patterns)

    # ld.add_action(launch_patterns)

    return ld