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
from collections import Counter

import numpy
import random

from communication_interfaces.msg import OpinionMessage
from ros2swarm.utils import setup_node
from ros2swarm.utils.vote_list import VoteList
from ros2swarm.voting_pattern.voting_pattern import VotingPattern
import datetime


class BeaconOpinionPattern(VotingPattern):
    """
    pattern_node >> communicate under the topic: vote_channel
    """

    def __init__(self):
        """Initialize the beacon opinion pattern node."""
        super().__init__('beacon_opinion_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('beacon_opinion_initial_field', "a"),
            ])

        self.param_initial_value = self.get_parameter(
            "beacon_opinion_initial_field").get_parameter_value().string_value

        print(str(self.get_namespace()))
        # get robot id
        self.id = super().get_robot_id()

        # set initial opinion
        self.opinion = self.param_initial_value

        # create reused OpinionMessage
        self.opinion_message = OpinionMessage()
        self.opinion_message.id = self.id
        self.opinion_message.opinion = self.opinion

        # list to store opinions
        self.opinion_list = []

        # OpinionMessage: {id[integer],opinion[integer]}
        self.broadcast_publisher = self.create_publisher(OpinionMessage,
                                                         '/beacon_broadcast',
                                                         10)

        self.opinion_publisher = self.create_publisher(OpinionMessage,
                                                       self.get_namespace() + '/opinion',
                                                       10)

        self.broadcast_subscription = self.create_subscription(
            OpinionMessage,
            '/beacon_broadcast',
            self.swarm_command_controlled(self.majority_broadcast_callback),
            10)

        self.first_broadcast_flag = False

    def timer_callback(self):
        """emit the own opinion."""
        self.get_logger().debug('Robot "{}" has opinion "{}""'
                                .format(self.get_namespace(), self.opinion))

        # emit opinion
        self.opinion_message.opinion = self.opinion
        self.broadcast_publisher.publish(self.opinion_message)
        self.opinion_publisher.publish(self.opinion_message)

        self.get_logger().debug('Robot "{}" send opinion "{}" at time "{}"'
                                .format(self.id, self.opinion, datetime.datetime.now()))
        self.first_broadcast_flag = True



def main(args=None):
    """Create a node for the majority rule pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, BeaconOpinionPattern)


if __name__ == '__main__':
    main()
