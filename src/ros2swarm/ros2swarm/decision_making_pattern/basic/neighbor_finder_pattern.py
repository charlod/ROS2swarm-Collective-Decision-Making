#!/usr/bin/env python3
#    Copyright 2023 Seray Arslan
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
from ros2swarm.utils import setup_node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from ros2swarm.abstract_pattern import AbstractPattern
from scipy.spatial import distance

class NeighborFinderPattern(AbstractPattern):
    def __init__(self):
        super().__init__('neighbor_finder_pattern')

        self.robot_positions = {}  # Store the position of each robot

        self.declare_parameters(
            namespace='',
            parameters=[
                ('num_of_neighbors', 0.0),
            ])
        
        self.model_states_subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        self.num_of_neighbors = int(self.get_parameter("num_of_neighbors").get_parameter_value().double_value)


    def model_states_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name.startswith('robot_name'):  # Check if the model name is a robot
                self.robot_positions[name] = [msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z]

    def find_closest_neighbors(self, robot_name, n=None, r=None):
        """Find the n closest neighbors to the given robot within radius r."""
        if robot_name not in self.robot_positions:
            return []

        robot_position = self.robot_positions[robot_name]
        distances = []

        for name, position in self.robot_positions.items():
            if name != robot_name:
                dist = distance.euclidean(robot_position, position)
                if r is None or dist <= r:
                    distances.append((name, dist))

        distances.sort(key=lambda x: x[1])  # Sort by distance

        if n is not None:
            return distances[:n]  # Return the n closest neighbors
        else:
            return distances

    def timer_callback(self):
        """Publish the configured twist message when called."""
        msg = Twist()
        msg.linear.x = self.param_x
        msg.angular.z = self.param_z
        self.command_publisher.publish(msg)
        self.get_logger().debug('Publishing {}:"{}"'.format(self.i, msg))

        self.i += 1

        for name in self.robot_positions.keys():
            neighbors = self.find_closest_neighbors(name, n=self.num_of_neighbors)
            self.get_logger().info('Closest neighbors to {}: {}'.format(name, neighbors))

def main(args=None):
    """
    Create a node for the neighbor finder pattern, spin it
    and handle the setup.
    """
    setup_node.init_and_spin(args, NeighborFinderPattern)


if __name__ == '__main__':
    main()
