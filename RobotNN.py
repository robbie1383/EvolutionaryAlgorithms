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
        len(self.network)
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
            fitness = 0
            while time.time() - start < 10:
                [Vl, Vr] = network.activations(robot.sensors)[1]
                readings = robot.moveFromVelocities(Vr, Vl, self.delta_t, self.room)
                visited.append(readings[1:])
                fitness = readings[0]
            # Calculate fitness given te movement readings
            print("collision times:", robot.collision)
            print("dust size:", len(robot.dust))
            print("cleaned  area size that close to wall ", len(robot.wall_close_dust))
            print("short moves", robot.short_move)
            print("move counts", robot.move_counter)
            print("-------------------------------")
            evaluations.append(fitness)
        print(evaluations)
        return evaluations

    def selection(self, evaluations):
        sorted = evaluations.copy()
        sorted.sort(reverse=True)
        selected = [self.population[evaluations.index(sorted[i])] for i in range(int(len(self.population) / 2))]

        return selected

    def reproduction(self, selected):
        children = []
        middle = int(len(selected))
        print("selected")
        for i in range (len(selected)):
            print(selected[i].network)
        for index in range(middle):
            children.append(selected[index])
            #print("old")
            #print(selected[index].network)
            selected[index].network[0] = np.add(selected[index].network[0],
                                ((np.random.random(), np.random.random(),np.random.random(),np.random.random(),
                                np.random.random(),np.random.random(),np.random.random(), np.random.random(),
                                np.random.random(),np.random.random(),np.random.random(),np.random.random(),
                                np.random.random(), np.random.random(),np.random.random(),np.random.random()),
                              (np.random.random(), np.random.random(), np.random.random(), np.random.random(),
                                np.random.random(), np.random.random(), np.random.random(), np.random.random(),
                                np.random.random(), np.random.random(), np.random.random(), np.random.random(),
                                np.random.random(), np.random.random(), np.random.random(), np.random.random()),
                              (np.random.random(), np.random.random(), np.random.random(), np.random.random(),
                                np.random.random(), np.random.random(), np.random.random(), np.random.random(),
                                np.random.random(), np.random.random(), np.random.random(), np.random.random(),
                                np.random.random(), np.random.random(), np.random.random(), np.random.random()),
                              (np.random.random(), np.random.random(), np.random.random(), np.random.random(),
                                np.random.random(), np.random.random(), np.random.random(), np.random.random(),
                                np.random.random(), np.random.random(), np.random.random(), np.random.random(),
                                np.random.random(), np.random.random(), np.random.random(), np.random.random())))
            selected[index].network[1]= np.add(selected[index].network[1],
                                ((np.random.random(), np.random.random(), np.random.random(), np.random.random()),
                                (np.random.random(), np.random.random(), np.random.random(),np.random.random())))
            #print("new")
            #print(selected[index].network)
            children.append(selected[index])
        print("children")
        for j in range (len(children)):
            print(children[j].network)
        return children

    def fitness(self):
        print("")












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
