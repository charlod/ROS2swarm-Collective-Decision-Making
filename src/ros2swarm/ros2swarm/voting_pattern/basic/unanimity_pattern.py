from collections import Counter

import numpy
import random

from communication_interfaces.msg import OpinionMessage
from ros2swarm.utils import setup_node
from ros2swarm.utils.vote_list import VoteList
from ros2swarm.voting_pattern.voting_pattern import VotingPattern
import datetime


class UnanimityPattern(VotingPattern):

    def __init__(self):
        super().__init__('unanimity_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('unanimity_initial_value', 0),
                ('unanimity_choose_start_value_at_random', True),
                ('unanimity_k', 0),
                ('unanimity_min_opinion', 0),
                ('unanimity_max_opinion', 0),
                ('unanimity_timer_period', 0.0),
            ])

        self.param_initial_value = self.get_parameter(
            "unanimity_initial_value").get_parameter_value().integer_value
        self.param_choose_start_value_at_random = self.get_parameter(
            "unanimity_choose_start_value_at_random").get_parameter_value().bool_value
        self.param_k = self.get_parameter(
            "unanimity_k").get_parameter_value().integer_value
        self.param_min_opinion = self.get_parameter(
            "unanimity_min_opinion").get_parameter_value().integer_value
        self.param_max_opinion = self.get_parameter(
            "unanimity_max_opinion").get_parameter_value().integer_value
        param_timer_period = self.get_parameter(
            "unanimity_timer_period").get_parameter_value().double_value

        print(str(self.get_namespace()))
        # get robot id
        self.id = super().get_robot_id()

        # set initial opinion
        if self.param_choose_start_value_at_random:
            self.opinion = numpy.random.randint(self.param_min_opinion,
                                                high=self.param_max_opinion)
        else:
            self.opinion = self.param_initial_value

        # create reused OpinionMessage
        self.opinion_message = OpinionMessage()
        self.opinion_message.id = self.id
        self.opinion_message.opinion = self.opinion

        # list to store opinions
        self.opinion_list = []

        # define time period to listen to other opinions
        self.timer = self.create_timer(param_timer_period, self.swarm_command_controlled_timer(self.timer_callback))

        # OpinionMessage: {id[integer],opinion[integer]}
        self.broadcast_publisher = self.create_publisher(OpinionMessage,
                                                         '/unanimity_broadcast',
                                                         10)

        self.opinion_publisher = self.create_publisher(OpinionMessage,
                                                       self.get_namespace() + '/opinion',
                                                       10)

        self.broadcast_subscription = self.create_subscription(
            OpinionMessage,
            '/unanimity_broadcast',
            self.swarm_command_controlled(self.unanimity_broadcast_callback),
            10)

        self.first_broadcast_flag = False

    def timer_callback(self):
        """Select a new opinion of another entity and emit the own opinion."""
        self.get_logger().debug('Robot "{}" has opinion "{}" and a list of size "{}"'
                                .format(self.get_namespace(), self.opinion,
                                        len(self.opinion_list)))
        self.get_logger().info('Robot "{}" has opinion "{}" and a list of size "{}"'
                                .format(self.get_namespace(), self.opinion,
                                        len(self.opinion_list)))
        # update opinion if at least one opinion were received and initial opinion send once
        if len(self.opinion_list) > 0 and self.first_broadcast_flag:
            self.get_logger().debug('Turtle "{}" reduce opinions "{}" at time "{}"'
                                    .format(self.id, self.opinion_list, datetime.datetime.now()))

            opinions = [e.opinion for e in self.opinion_list]
            # find max opinion
            distribution = Counter(opinions).most_common()
            # check the maximum is reached by more than one opinion
            maximum = distribution[0][1]
            maxima = []
            for e in distribution:
                if e[1] == maximum:
                    maxima.append(e[0])
                else:
                    # the input is ordered so no need for further search
                    break
            # choose randomly one of the maxima
            if random.choice(maxima) == self.param_k:
                self.opinion = random.choice(maxima)

            self.get_logger().debug('Robot "{}" reduce opinions "{}"->"{}"'
                                    .format(self.id, opinions, self.opinion))

            self.opinion_list = []

        # emit opinion
        self.opinion_message.opinion = self.opinion
        self.broadcast_publisher.publish(self.opinion_message)
        self.opinion_publisher.publish(self.opinion_message)

        self.get_logger().debug('Robot "{}" send opinion "{}" at time "{}"'
                                .format(self.id, self.opinion, datetime.datetime.now()))
        self.first_broadcast_flag = True

    def unanimity_broadcast_callback(self, opinion_msg):
        """Store heard opinion message in a list to use it later."""

        self.get_logger().debug('Robot "{}" got opinion "{}" with list "{}" at time "{}"'
                                .format(self.id, opinion_msg, self.opinion_list,
                                        datetime.datetime.now()))

        self.opinion_list = VoteList.update_opinion(self.opinion_list, opinion_msg, self.id)


def main(args=None):
    """Create a node for the k-unanimity rule pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, UnanimityPattern)


if __name__ == '__main__':
    main()
