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

from geometry_msgs.msg import Twist
import random
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils import setup_node
from communication_interfaces.msg import RangeData
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.utils.state import State


class RandomWalkPattern(MovementPattern):

    def __init__(self):
        """Initialize the random walk pattern node."""
        super().__init__('random_walk')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('random_walk_linear', 0.0),
                ('random_walk_angular', 0.0),
                ('random_walk_timer_period', 0.0),
                ('random_walk_rot_interval', 0.0),
                ('random_walk_lin_interval_min', 0.0),
                ('random_walk_lin_interval_max', 0.0)
            ])

        self.walk = self.create_timer(5, self.swarm_command_controlled_timer(self.random))
        self.timer = self.create_timer(
            self.get_parameter("random_walk_timer_period").get_parameter_value().double_value,
            self.swarm_command_controlled_timer(self.timer_callback))
        self.param_x = float(self.get_parameter("random_walk_linear").get_parameter_value().double_value)
        self.param_z = float(
            self.get_parameter("random_walk_angular").get_parameter_value().double_value)
        self.rot_interval = float(self.get_parameter("random_walk_rot_interval").get_parameter_value().double_value)
        self.lin_interval_min = float(self.get_parameter("random_walk_lin_interval_min")
                                      .get_parameter_value().double_value)
        self.lin_interval_max = float(self.get_parameter("random_walk_lin_interval_max")
                                      .get_parameter_value().double_value)
        self.i = 0
        self.turn = False
        self.current_msg = Twist()


        self.range_data_subscription = self.create_subscription(
            RangeData,
            self.get_namespace() + '/bump_front_center_plugin',
            self.swarm_command_controlled(self.range_data_callback),
            qos_profile=qos_profile_sensor_data
        )

        # import rospy
        # from controller_manager_msgs.srv import ListControllers

        # __list_controllers = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
        # print("*************************************************************")

        # print(__list_controllers)
        # __list_controllers.call() #whenever required


    def timer_callback(self):
        """Publish the configured twist message when called."""
        self.command_publisher.publish(self.current_msg)
        self.get_logger().debug('Publishing {}:"{}"'.format(self.i, self.current_msg))
        self.i += 1

    def random(self):
        """Randomly change direction."""
        msg = Twist()
        if self.turn:
            sign = 1 if random.random() < 0.5 else -1
            msg.angular.z = random.uniform(self.param_z / 5, 4 * self.param_z / 5) * sign  # Richtung?
            msg.linear.x = 0.0
            self.walk.cancel()
            self.walk = self.create_timer(random.uniform(0, self.rot_interval), self.random)
        else:
            msg.angular.z = 0.0
            msg.linear.x = self.param_x
            self.walk.cancel()
            bu = random.uniform(self.lin_interval_min, self.lin_interval_max)
            self.walk = self.create_timer(bu, self.random)
            self.get_logger().info('Publishing {}:"{}"'.format(self.i, bu))
        self.turn = not self.turn
        self.current_msg = msg


    def range_data_callback(self, incoming_msg):
        """Call back if a new scan msg is available."""
        direction = self.vector_calc(incoming_msg)
        self.command_publisher.publish(direction)

    def vector_calc(self, current_range):
        """Calculate the direction vector for the current scan.

        # INIT: determine if next state is EXPLORE or JOIN_GROUP
        # EXPLORE: -> move forward
        # STAY: -> num robots
        # JOIN_GROUP: -> robot moves towards center of mass [?? to nearest]
        # LEAVE_GROUP: -> robot moves away from center of mass # also has timer OR until no bots detected
        """
        if current_range is None:
            return Twist()

        self.get_logger().debug('Robot "{}" is in state "{}"'.format(self.get_namespace(), self.state))
        result = Twist()

        if self.state is State.INIT:
            robots, _ = self.identify_robots(current_range)
            if robots:
                self.state = State.JOIN_GROUP
            else:
                self.state = State.EXPLORE
        elif self.state is State.EXPLORE:
            result, self.state = self.explore(current_range)
        elif self.state is State.JOIN_GROUP:
            result, self.state = self.join_group(current_range)
        elif self.state is State.STAY_IN_GROUP:
            result, self.state = self.stay_in_group(current_range)
        elif self.state is State.LEAVE_GROUP:
            result, self.state = self.leave_group(current_range)

        return result

def main(args=None):
    setup_node.init_and_spin(args, RandomWalkPattern)


if __name__ == '__main__':
    main()
