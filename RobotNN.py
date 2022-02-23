import numpy as np

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





nn = RobotNN()