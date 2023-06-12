from geometry_msgs.msg import Twist, Point
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils import setup_node
from gazebo_msgs.msg import ModelStates
from scipy.spatial import distance


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
                ('drive_linear', 0.0),
                ('drive_angular', 0.0),
            ])

        timer_period = float(
            self.get_parameter("drive_timer_period").get_parameter_value().double_value)
        self.timer = self.create_timer(timer_period, self.swarm_command_controlled_timer(self.timer_callback))
        self.i = 0
        self.param_x = float(self.get_parameter("drive_linear").get_parameter_value().double_value)
        self.param_z = float(
            self.get_parameter("drive_angular").get_parameter_value().double_value)

        self.robot_positions = {}  # Store the position of each robot

        self.model_states_subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

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
            neighbors = self.find_closest_neighbors(name, n=3)  # Find the 3 closest neighbors
            self.get_logger().info('Closest neighbors to {}: {}'.format(name, neighbors))


def main(args=None):
    """
    Create a node for the drive pattern, spin it
    and handle the setup.
    """
    setup_node.init_and_spin(args, DrivePattern)


if __name__ == '__main__':
    main()
