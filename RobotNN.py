import numpy as np
from Robot import Robot
import time


class RobotNN():

    def __init__(self, network):
        if network is None:
            self.network = []
            hiddenLayer = [np.random.rand(1, 16)[0] / 10 for i in range(4)]  # size + 1 for the bias
            outputLayer = [np.random.rand(1, 4)[0] / 10 for i in range(2)]  # size + 1 for the bias
            self.network.append(hiddenLayer)
            self.network.append(outputLayer)
            self.feedback = [0, 0, 0, 0]
        else :
            self.network = network
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
        self.population = [RobotNN(None) for i in range(10)]
        self.room = room
        self.delta_t = delta_t
        self.initPosition = initPosition

    def evolve(self):
        evaluations = self.evaluate()
        selected, best = self.selection(evaluations)
        children = self.reproduction(selected)
        self.population = children

        return np.min(evaluations), np.mean(evaluations), best

    def evaluate(self):
        evaluations = []
        for network in self.population:
            # Simulate robot movement for 30 seconds
            robot = Robot(self.room, self.initPosition, 60)
            start = time.time()
            visited = []
            fitness = 0
            while time.time() - start < 10:
                [Vl, Vr] = network.activations(robot.sensors)[1]
                readings = robot.moveFromVelocities(Vr, Vl, self.delta_t, self.room)
                visited.append(readings[1:])
                fitness = readings[0]
            # Calculate fitness given te movement readings
            """
            print("collision times:", robot.collision)
            print("dust size:", len(robot.dust))
            print("cleaned  area size that close to wall ", len(robot.wall_close_dust))
            print("short moves", robot.short_move)
            print("move counts", robot.move_counter)
            print("-------------------------------")
            """
            evaluations.append(fitness)
        # print(evaluations)
        return evaluations

    def selection(self, evaluations):
        sorted = evaluations.copy()
        sorted.sort(reverse=True)
        selected = [self.population[evaluations.index(sorted[i])] for i in range(int(len(self.population) / 2))]
        best = selected[0].network
        return selected, best

    def reproduction(self, selected):
        children = []
        children += selected
        for i in range(len(selected)):
            aux = []
            for layer in range(len(selected[0].network)):
                aux.append(np.mean([selected[0].network[layer], selected[i].network[layer]], 0))
            children.append(RobotNN(aux))
        return children
"""
    def reproduction(self, selected):
        children = []
        middle = int(len(selected))
        
        for index in range(middle):
            children.append(selected[index])

            selected[index].network[0] = np.add(selected[index].network[0],
                                                ((np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random(),
                                                  np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random(),
                                                  np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random(),
                                                  np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random()),
                                                 (np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random(),
                                                  np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random(),
                                                  np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random(),
                                                  np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random()),
                                                 (np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random(),
                                                  np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random(),
                                                  np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random(),
                                                  np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random()),
                                                 (np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random(),
                                                  np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random(),
                                                  np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random(),
                                                  np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random())))
            selected[index].network[1] = np.add(selected[index].network[1],
                                                ((np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random()),
                                                 (np.random.random(), np.random.random(), np.random.random(),
                                                  np.random.random())))

            children.append(selected[index])

        return children
"""

