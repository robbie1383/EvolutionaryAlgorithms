import numpy as np
from Robot import Robot

class RobotNN():

    def __init__(self):
        self.network = []
        hiddenLayer = [np.random.rand(1, 16)[0] for i in range(4)]  # size + 1 for the bias
        outputLayer = [np.random.rand(1, 4)[0] for i in range(2)]  # size + 1 for the bias
        self.network.append(hiddenLayer)
        self.network.append(outputLayer)

        self.feedback = []

    def activations(self, input):
        activations = []
        for layer in range(len(self.network)):
            layera = []
            for unit in range(len(self.network[layer])):
                a = 0
                for i in range(len(self.network[layer][0])):
                    a += input[i] * self.network[layer][unit][i]
                layera.append(a)
            activations.append(layera)
            input = activations[layer]

        return activations


class RobotEA():

    def __init__(self):
        self.population = [RobotNN() for i in range(10)]

    def evolve(self):
        evaluations = self.evaluate()
        selected = self.selection(evaluations)
        children = self.reproduction(selected)
        self.population = children

        return np.min(evaluations), np.mean(evaluations)

    def evaluate(self):
        evaluations = []

        for network in self.population:
            robot = Robot()
            # simulate robot movement for a certain amount of time
            # calculate fitness given that movement

        return evaluations

    def selection(self, evaluations):
        sorted = evaluations.copy()
        sorted.sort(reverse=True)
        selected = [self.population[evaluations.index(sorted[i])] for i in range(len(self.population)/2)]
        return selected

    def reproduction(self, selected):
        children = []

        return children

