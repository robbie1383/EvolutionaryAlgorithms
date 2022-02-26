import numpy as np
from Robot import Robot
import time


class RobotNN():

    def __init__(self):
        self.network = []
        hiddenLayer = [np.random.rand(1, 16)[0] / 10 for i in range(4)]  # size + 1 for the bias
        outputLayer = [np.random.rand(1, 4)[0] / 10 for i in range(2)]  # size + 1 for the bias
        self.network.append(hiddenLayer)
        self.network.append(outputLayer)

        self.feedback = [0, 0, 0, 0]

    def activations(self, input):
        activations = []
        input += self.feedback
        for layer in range(len(self.network)):
            layera = []
            for unit in range(len(self.network[layer])):
                a = 0
                for i in range(len(self.network[layer][0])):
                    a += input[i] * self.network[layer][unit][i]
                layera.append(a)
            activations.append(layera)
            input = activations[layer]

        self.feedback = activations[0]
        return activations


class RobotEA():

    def __init__(self, room, delta_t, initPosition):
        self.population = [RobotNN() for i in range(10)]
        self.room = room
        self.delta_t = delta_t
        self.initPosition = initPosition

    def evolve(self):
        evaluations = self.evaluate()
        selected = self.selection(evaluations)
        children = self.reproduction(selected)
        self.population = children

        return np.min(evaluations), np.mean(evaluations)

    def evaluate(self):
        evaluations = []

        for network in self.population:
            # Simulate robot movement for 30 seconds
            robot = Robot(self.room, self.initPosition, 60)
            start = time.time()
            visited = []
            collisions = 0
            while time.time() - start < 30:
                [Vl, Vr] = network.activations(robot.sensors)[1]
                readings = robot.moveFromVelocities(Vr, Vl, self.delta_t, self.room)
                visited.append(readings[1:2])
                if readings[0] == "Danger!":
                    collisions += 1
                print(visited)

            # Calculate fitness given te movement readings
        print(evaluations)
        return evaluations

    def selection(self, evaluations):
        sorted = evaluations.copy()
        sorted.sort(reverse=True)
        selected = [self.population[evaluations.index(sorted[i])] for i in range(int(len(self.population) / 2))]
        return selected

    def reproduction(self, selected):
        children = []
        # Florene
        return children


sev = 35  # SCREEN_EDGE_VACANCY
w = 900
h = 900
line0 = [(sev, sev), (sev, h - sev), (w - sev, h - sev), (w - sev, sev), (sev, sev)]
line1 = [(300, sev), (300, 750)]
line2 = [(600, sev), (600, 300)]
line3 = [(600, 450), (600, h - sev)]
room1 = [line0, line1, line2, line3]
initPosition = [400, 600]
delta_t = 0.01

test = RobotEA(room1, delta_t, initPosition)
test.evolve()
