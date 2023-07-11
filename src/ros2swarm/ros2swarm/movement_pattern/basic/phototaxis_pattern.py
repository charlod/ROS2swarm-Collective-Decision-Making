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
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils import setup_node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class PhototaxisPattern(MovementPattern):
    def __init__(self):
        """Initialize the phototaxis pattern node."""
        super().__init__('phototaxis')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('phototaxis_linear', 0.0),
                ('phototaxis_angular', 0.0),
                ('phototaxis_timer_period', 0.0),
            ])

        self.bridge = CvBridge()

        # Subscribe to the camera image topic
        self.camera_subscriber = self.create_subscription(
            Image,
            'camera_topic',  # TODO
            self.camera_callback,
            10
        )
        self.timer = self.create_timer(
            self.get_parameter("phototaxis_timer_period").get_parameter_value().double_value,
            self.swarm_command_controlled_timer(self.timer_callback))

        self.param_x = float(self.get_parameter("phototaxis_linear").get_parameter_value().double_value)
        self.param_z = float(self.get_parameter("phototaxis_angular").get_parameter_value().double_value)

        self.current_msg = Twist()

    def camera_callback(self, msg):
        """Process the camera image and adjust the movement direction."""
        # Convert the ROS Image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert the image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Find the coordinates of the brightest pixel
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)

        # Get the width of the image
        width = gray.shape[1]

        twist = Twist()

        # Determine the direction to turn based on the location of the brightest pixel
        if maxLoc[0] < width / 2:
            # The brightest pixel is in the left half of the image
            twist.angular.z = self.angular_speed  # Turn left for PhototaxisPattern, right for AntiphototaxisPattern
        else:
            # The brightest pixel is in the right half of the image
            twist.angular.z = -self.angular_speed  # Turn right for PhototaxisPattern, left for AntiphototaxisPattern

        # Publish the Twist message
        self.command_publisher.publish(twist)

    def timer_callback(self):
        """Publish the configured twist message when called."""
        self.command_publisher.publish(self.current_msg)
        self.get_logger().debug('Publishing {}:"{}"'.format(self.i, self.current_msg))
        self.i += 1

def main(args=None):
    setup_node.init_and_spin(args, PhototaxisPattern)


if __name__ == '__main__':
    main()
