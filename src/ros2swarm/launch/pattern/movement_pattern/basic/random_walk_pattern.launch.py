#!/usr/bin/env python3
#    Copyright 2021 Tavia Plattenteich
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.
import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import os
import sys
import launch_ros.actions

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Start the nodes required for the drive pattern."""

    robot_namespace = LaunchConfiguration('robot_namespace', default='robot1')
    # config_dir = LaunchConfiguration('config_dir', default='config_dir_default')
    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config', 'turtlebot4')
    log_level = LaunchConfiguration("log_level", default='debug')

    ld = LaunchDescription()
    ros2_pattern_node = launch_ros.actions.Node(
        package='ros2swarm',
        executable='random_walk_pattern',
        namespace=robot_namespace,
        output='screen',
        parameters=[
            PathJoinSubstitution([config_dir, 'movement_pattern', 'basic', 'random_walk_pattern.yaml'])],
        arguments=['--ros-args', '--log-level', log_level]
    )
    ld.add_action(ros2_pattern_node)

    return ld
