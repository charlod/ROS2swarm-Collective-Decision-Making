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
from geometry_msgs.msg import Twist
import random
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils import setup_node
from communication_interfaces.msg import OpinionMessage

class BinaryDiscriminationExplorationPattern(MovementPattern):
    """
    Combined movement pattern that switches between random walk, phototaxis, and antiphototaxis
    based on the input from beacon robots.
    """

    def __init__(self):
        """Initialize the combined pattern node."""
        super().__init__('combined_pattern')

        # Initialize the state to 'random_walk'
        self.state = 'random_walk'

        # Subscriptions for the three patterns
        self.random_walk_sub = self.create_subscription(
            Twist,
            self.get_namespace() + '/drive_command_random_walk',
            self.command_callback_random_walk,
            10)
        self.phototaxis_sub = self.create_subscription(
            Twist,
            self.get_namespace() + '/drive_command_phototaxis',
            self.command_callback_phototaxis,
            10)
        self.antiphototaxis_sub = self.create_subscription(
            Twist,
            self.get_namespace() + '/drive_command_antiphototaxis',
            self.command_callback_antiphototaxis,
            10)

        # Subscription for the beacon input
        self.beacon_sub = self.create_subscription(
            OpinionMessage,
            '/beacon_broadcast',
            self.beacon_callback,
            10)

        # Latest commands from the three patterns
        self.latest_random_walk_cmd = Twist()
        self.latest_phototaxis_cmd = Twist()
        self.latest_antiphototaxis_cmd = Twist()

    def command_callback_random_walk(self, cmd):
        """Store the latest command from the random walk pattern."""
        self.latest_random_walk_cmd = cmd

    def command_callback_phototaxis(self, cmd):
        """Store the latest command from the phototaxis pattern."""
        self.latest_phototaxis_cmd = cmd

    def command_callback_antiphototaxis(self, cmd):
        """Store the latest command from the antiphototaxis pattern."""
        self.latest_antiphototaxis_cmd = cmd

    def beacon_callback(self, msg):
        """Switch the state based on the beacon input."""
        if msg.opinion == 'a':
            self.state = 'phototaxis'
        elif msg.opinion == 'b':
            self.state = 'antiphototaxis'
        else:
            self.state = 'random_walk'

    def timer_callback(self):
        """Publish the command from the current pattern."""
        if self.state == 'random_walk':
            self.command_publisher.publish(self.latest_random_walk_cmd)
        elif self.state == 'phototaxis':
            self.command_publisher.publish(self.latest_phototaxis_cmd)
        elif self.state == 'antiphototaxis':
            self.command_publisher.publish(self.latest_antiphototaxis_cmd)

def main(args=None):
    """
    Create a node for the neighbor finder pattern, spin it
    and handle the setup.
    """
    setup_node.init_and_spin(args, BinaryDiscriminationExplorationPattern)


if __name__ == '__main__':
    main()
