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
from communication_interfaces.msg import RangeData
from collections import Counter

from ros2swarm.abstract_pattern import AbstractPattern
from scipy.spatial import distance
from communication_interfaces.msg import OpinionMessage
from ros2swarm.utils import setup_node
from ros2swarm.utils.vote_list import VoteList
from ros2swarm.voting_pattern.voting_pattern import VotingPattern
import random
import cv2


class BinaryDiscriminationPattern(AbstractPattern):
    def __init__(self):
        """Initialize the binary discrimination pattern node."""
        super().__init__('binary_discrimination_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('binary_discrimination_initial_opinion', 'a'),
                ('binary_discrimination_timer_period', 0.0),
            ])

        self.param_initial_opinion = self.get_parameter(
            "binary_discrimination_initial_opinion").get_parameter_value().string_value
        param_timer_period = self.get_parameter(
            "binary_discrimination_timer_period").get_parameter_value().double_value

        self.opinion = self.param_initial_opinion
        self.opinion_message = OpinionMessage()
        self.opinion_message.id = self.id
        self.opinion_message.opinion = self.opinion

        # list to store opinions
        self.opinion_list = []

        self.timer = self.create_timer(param_timer_period, self.swarm_command_controlled_timer(self.timer_callback))

        self.opinion_publisher = self.create_publisher(OpinionMessage,
                                                       self.get_namespace() + '/opinion',
                                                       10)

        self.opinion_subscription = self.create_subscription(
            OpinionMessage,
            '/opinion',
            self.opinion_callback,
            10)


        self.random_walk_subscriber = self.create_subscription(
            Twist,
            'random_walk/drive_command',  
            self.random_walk_callback,
            10
        )
        self.phototaxis_subscriber = self.create_subscription(
            Twist,
            'phototaxis/drive_command',  
            self.phototaxis_callback,
            10
        )
        self.antiphototaxis_subscriber = self.create_subscription(
            Twist,
            'antiphototaxis/drive_command',  
            self.antiphototaxis_callback,
            10
        )

        self.ir_subscriber = self.create_subscription(
            RangeData,
            'ir_ground_sensor_topic',  # TODO
            self.ir_callback,
            10
        )
        self.current_location = 'nest'  # Initial location

    def model_states_callback(self, msg):
        # Update the robot's position
        for i, name in enumerate(msg.name):
            if name == 'robot_name':  # Replace with the actual robot name
                self.robot_position = [msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z]

    def camera_callback(self, msg):
        # Convert the image message to a numpy array
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Find the brightest pixel
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))

        # Update the light source position
        self.light_source_position = max_loc

        # update opinion if at least one opinion were received
        if len(self.opinion_list) > 0:
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
            self.opinion = random.choice(maxima)

            self.get_logger().debug('Robot "{}" reduce opinions "{}"->"{}"'
                                    .format(self.id, opinions, self.opinion))

            self.opinion_list = []

        # emit opinion
        self.opinion_message.opinion = self.opinion
        self.opinion_publisher.publish(self.opinion_message)

        self.get_logger().debug('Robot "{}" send opinion "{}"'
                                .format(self.id, self.opinion))

    def opinion_callback(self, opinion_msg):
        """Store heard opinion message in a list to use it later."""
        self.get_logger().debug('Robot "{}" got opinion "{}" with list "{}"'
                                .format(self.id, opinion_msg, self.opinion_list))

        self.opinion_list.append(opinion_msg)

    def random_walk_callback(self, msg):
        self.random_walk_command = msg

    def phototaxis_callback(self, msg):
        self.phototaxis_command = msg

    def antiphototaxis_callback(self, msg):
        self.antiphototaxis_command = msg


    def timer_callback(self):
        # Decide which pattern to use based on the current state and other conditions
        if self.state == 'exploration':
            if self.opinion == 'a':
                self.current_msg = self.phototaxis_command
            else:  # self.opinion == 'b'
                self.current_msg = self.antiphototaxis_command
        else:  # self.state == 'dissemination'
            self.current_msg = self.random_walk_command

        # Publish the current movement command
        self.command_publisher.publish(self.current_msg)

        # Update the robot's opinion based on the opinions of other robots
        self.update_opinion()


    def timer_callback(self):
        """Select a new opinion of another entity and emit the own opinion."""
        self.get_logger().debug('Robot "{}" has opinion "{}"'
                                .format(self.get_namespace(), self.opinion))

        # Emit opinion
        self.opinion_message.opinion = self.opinion
        self.opinion_publisher.publish(self.opinion_message)

        # Control the robot's movement based on its current opinion and the location of the light source
        if self.opinion == 'a' and self.light_source_position is not None:
            if self.robot_position[0] < self.light_source_position[0]:  # The robot is to the left of the light source
                self.phototaxis_publisher.publish(Twist())  # Replace with the actual phototaxis command
            else:  # The robot is to the right of the light source
                self.antiphototaxis_publisher.publish(Twist())  # Replace with the actual antiphototaxis command
        elif self.opinion == 'b' and self.light_source_position is not None:
            if self.robot_position[0] > self.light_source_position[0]:  # The robot is to the right of the light source
                self.phototaxis_publisher.publish(Twist())  # Replace with the actual phototaxis command
            else:  # The robot is to the left of the light source
                self.antiphototaxis_publisher.publish(Twist())  # Replace with the actual antiphototaxis command
        else:
            self.random_walk_publisher.publish(Twist())  # Replace with the actual random walk command