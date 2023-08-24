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
from ros2swarm.abstract_pattern import AbstractPattern
from ros2swarm.utils import setup_node
import importlib

class ContinuousExecutionPattern(AbstractPattern):
    """
    A pattern for executing patterns continuously.
    """

    def __init__(self):
        """Initialize the continuous execution pattern."""
        super().__init__('continuous_execution_pattern')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('exploration_pattern_module', ''),
                ('exploration_pattern_class', ''),
                ('opinion_switch_pattern_module', ''),
                ('opinion_switch_pattern_class', ''),
            ])

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


def main(args=None):
    """
    Create a node for the continuous execution pattern, spin it
    and handle the setup.
    """
    setup_node.init_and_spin(args, ContinuousExecutionPattern)


if __name__ == '__main__':
    main()
