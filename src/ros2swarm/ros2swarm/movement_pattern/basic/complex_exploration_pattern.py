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
import random
from ros2swarm.movement_pattern import MovementPattern
from ros2swarm.utils import setup_node
from geometry_msgs.msg import Twist
from communication_interfaces.msg import RangeData

class ComplexExplorationPattern(MovementPattern):

    def __init__(self):
        """Initialize the complex exploration pattern node."""
        super().__init__('complex_exploration')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('timer_period', 0.5),
                ('linear_speed', 0.2),
                ('angular_speed', 0.5)
            ])

        # Subscribe to the IR ground sensor topics
        self.left_ir_subscriber = self.create_subscription(
            RangeData,
            'left_ir_ground_sensor_topic',  # Replace with the actual topic name
            self.left_ir_callback,
            10
        )
        self.right_ir_subscriber = self.create_subscription(
            RangeData,
            'right_ir_ground_sensor_topic',  # Replace with the actual topic name
            self.right_ir_callback,
            10
        )

        # Timer for controlling the movement
        timer_period = self.get_parameter("timer_period").get_parameter_value().double_value
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # State variables
        self.left_ground_color = None
        self.right_ground_color = None

        # Dictionary to keep count of tiles visited
        self.tiles_visited = {"black": 0, "white": 0}

    def left_ir_callback(self, msg):
        """Callback for processing left IR ground sensor data."""
        # Process sensor data and store the ground color.
        self.left_ground_color = "white" if msg.range > threshold else "black"

    def right_ir_callback(self, msg):
        """Callback for processing right IR ground sensor data."""
        # Process sensor data and store the ground color.
        self.right_ground_color = "white" if msg.range > threshold else "black"

    def timer_callback(self):
        """Publish the twist message when called."""
        # Combine random walk with ground evaluation logic here.
        # Publish commands to the robot based on logic.
        twist = Twist()

        # Example logic: If on a black tile, turn, otherwise go straight
        if self.left_ground_color == "black" or self.right_ground_color == "black":
            twist.angular.z = random.uniform(-self.get_parameter("angular_speed").get_parameter_value().double_value,
                                             self.get_parameter("angular_speed").get_parameter_value().double_value)
            # Increment the count of black tiles visited
            self.tiles_visited["black"] += 1
        else:
            twist.linear.x = self.get_parameter("linear_speed").get_parameter_value().double_value
            # Increment the count of white tiles visited
            self.tiles_visited["white"] += 1

        # Publish the twist message
        self.command_publisher.publish(twist)

        # Log the count of tiles visited
        self.get_logger().info(f"Tiles visited: {self.tiles_visited}")

# Additional methods
# ...

# You need to define a threshold value to differentiate between black and white.
# This threshold should be determined empirically by observing the values published by the IR sensors
# on black and white surfaces.
threshold = SOME_VALUE  # Replace SOME_VALUE with an appropriate threshold

def main(args=None):
    """Create a node for the discussed dispersion pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, ComplexExplorationPattern)


if __name__ == '__main__':
    main()
