from geometry_msgs.msg import Twist, Point
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils import setup_node

class DrivePattern(MovementPattern):
    """
    A simple pattern for driving a constant direction vector.

    Which is configured with the parameters of this pattern.
    How often the direction is published is configured in the timer period parameter.
    """

    def __init__(self):
        """Initialize the drive pattern."""
        super().__init__('drive_pattern')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('drive_timer_period', 0.0),
                ('drive_linear', 5.0),
                ('drive_angular', 0.0),
            ])

        timer_period = float(
            self.get_parameter("drive_timer_period").get_parameter_value().double_value)
        self.timer = self.create_timer(timer_period, self.swarm_command_controlled_timer(self.timer_callback))
        self.i = 0
        self.param_x = float(self.get_parameter("drive_linear").get_parameter_value().double_value)
        self.param_z = float(
            self.get_parameter("drive_angular").get_parameter_value().double_value)


    def timer_callback(self):
        """Publish the configured twist message when called."""
        msg = Twist()
        msg.linear.x = self.param_x
        msg.angular.z = self.param_z
        self.command_publisher.publish(msg)
        self.get_logger().debug('Publishing {}:"{}"'.format(self.i, msg))

def main(args=None):
    """
    Create a node for the drive pattern, spin it
    and handle the setup.
    """
    setup_node.init_and_spin(args, DrivePattern)


if __name__ == '__main__':
    main()
