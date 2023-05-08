from ros2swarm.decision_making import DecisionMaking

#TODO

class ContinuousExecution(DecisionMaking):
    def __init__(self, explore_algorithm, disseminate_algorithm, opinion_update_algorithm):
        self.explore_algorithm = explore_algorithm
        self.disseminate_algorithm = disseminate_algorithm
        self.opinion_update_algorithm = opinion_update_algorithm

    def run(self):
        while(1):
            self.explore_algorithm.execute()
            self.disseminate_algorithm.execute()
            self.opinion_update_algorithm.execute()

    def explore(self):
        pass

    def disseminate(self):
        pass

    def opinion_update(self):
        pass