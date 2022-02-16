import numpy as np

class EA:

    def __init__(self, size, r, optimization):
        self.r = r
        self.population = np.array([(np.random.uniform(-r, r), np.random.uniform(-r, r)) for i in range(size)])
        self.optimization = optimization

    def evolve(self):
        evaluations = self.evaluate()
        selected = self.selection(evaluations)
        children = self.crossover(selected)
        self.population = self.checkBounds(children)
        return np.min(evaluations), np.mean(evaluations)

    def evaluate(self):
        minimum = self.optimization(0, 0)
        evaluations = [self.optimization(particle[0], particle[1]) - minimum for particle in self.population]
        return evaluations

    def selection(self, evaluations):
        chosen = 5
        sorted = evaluations.copy()
        sorted.sort()
        selected = []
        for i in range(chosen):
            selected.append(self.population[evaluations.index(sorted[i])])
        return selected

    def reproduction(self, selected):
        children = [*selected, *selected]
        return children

    def crossover(self, selected):
        children = []
        middle = int(len(selected))
        for index in range(middle):
            children.append(selected[index])
            newChild = np.add(selected[index], (np.random.random(), np.random.random()))
            children.append(newChild)
        return children

    def checkBounds(self, children):
        for child in children:
            child[0] = max(-self.r, min(self.r, child[0]))
            child[1] = max(-self.r, min(self.r, child[1]))
        return children