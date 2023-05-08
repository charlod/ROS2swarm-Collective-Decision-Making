from ros2swarm.decision_making import DecisionMaking

#TODO
class ContinuousExecution(DecisionMaking):
    def __init__(self, explore_algorithm, disseminate_algorithm, opinion_update_algorithm):
        self.explore_algorithm = explore_algorithm
        self.disseminate_algorithm = disseminate_algorithm
        self.opinion_update_algorithm = opinion_update_algorithm

    def explore(self):
        self.explore_algorithm.execute()

    def disseminate(self):
        self.disseminate_algorithm.execute()

    def opinion_update(self):
        self.opinion_update_algorithm.execute()