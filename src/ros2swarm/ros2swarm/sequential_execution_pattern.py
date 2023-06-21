from ros2swarm.abstract_pattern import AbstractPattern
from ros2swarm.utils import setup_node
import importlib

class SequentialExecutionPattern(AbstractPattern):
    """
    A pattern for executing patterns sequentially.
    """

    def __init__(self):
        """Initialize the sequential execution pattern."""
        super().__init__('sequential_execution_pattern')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('sequential_timer_period', 0.0),
                ('exploration_pattern_module', ''),
                ('exploration_pattern_class', ''),
                ('opinion_switch_pattern_module', ''),
                ('opinion_switch_pattern_class', ''),
            ])

        timer_period = float(
            self.get_parameter("sequential_timer_period").get_parameter_value().double_value)
        self.timer = self.create_timer(timer_period, self.swarm_command_controlled_timer(self.timer_callback))
        
        # Get pattern module and class names from parameters
        self.exploration_pattern_module = self.get_parameter('exploration_pattern_module').get_parameter_value().string_value
        self.exploration_pattern_class = self.get_parameter('exploration_pattern_class').get_parameter_value().string_value
        self.opinion_switch_pattern_module = self.get_parameter('opinion_switch_pattern_module').get_parameter_value().string_value
        self.opinion_switch_pattern_class = self.get_parameter('opinion_switch_pattern_class').get_parameter_value().string_value


        print(f"Exploration pattern module: {self.exploration_pattern_module}")
        print(f"Exploration pattern class: {self.exploration_pattern_class}")


        # Dynamically import and instantiate patterns
        self.exploration_pattern_instance = getattr(importlib.import_module(self.exploration_pattern_module), self.exploration_pattern_class)()
        self.opinion_switch_pattern_instance = getattr(importlib.import_module(self.opinion_switch_pattern_module), self.opinion_switch_pattern_class)()

        self.current_phase = "exploration"
        self.execute_exploration_pattern()

    def get_pattern_class(self, pattern_module, pattern_class):
        try:
            pattern_module = importlib.import_module(pattern_module)
            pattern_class = getattr(pattern_module, pattern_class)
            return pattern_class
        except ImportError:
            self.get_logger().error(f"Pattern module {pattern_module} could not be imported.")
            return None
        except AttributeError:
            self.get_logger().error(f"Pattern module {pattern_module} does not have a class named {pattern_class}.")
            return None
        
    def execute_exploration_pattern(self):
        print("Executing exploration pattern...")
        exploration_pattern_class = self.get_pattern_class(self.exploration_pattern_module, self.exploration_pattern_class)
        self.exploration_pattern_instance = exploration_pattern_class()

    def execute_opinion_switch_pattern(self):
        opinion_switch_pattern_class = self.get_pattern_class(self.opinion_switch_pattern_module, self.opinion_switch_pattern_class)
        self.opinion_switch_pattern_instance = opinion_switch_pattern_class()

    def timer_callback(self):
        if self.current_phase == "exploration":
            self.destroy_timer(self.exploration_pattern_instance.timer)
            self.execute_opinion_switch_pattern()
            self.current_phase = "opinion_switch"
        else:
            self.destroy_timer(self.opinion_switch_pattern_instance.timer)
            self.execute_exploration_pattern()
            self.current_phase = "exploration"


def main(args=None):
    """
    Create a node for the sequential execution pattern, spin it
    and handle the setup.
    """
    setup_node.init_and_spin(args, SequentialExecutionPattern)

if __name__ == '__main__':
    main()
