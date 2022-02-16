import os
import imageio
from EA import EA
import numpy as np
from matplotlib import pyplot as plt

def rosenbrock(x, y):
    a = 0
    b = 10
    return (a - x) ** 2 + b * (y - x ** 2) ** 2

def rastrigin(x, y):
    return 10 * 2 + (x ** 2 - 10 * np.cos(2 * np.pi * x)) + (y ** 2 - 10 * np.cos(2 * np.pi * y))

def plotFunction(range, evaluation):
    steps = 70
    x = np.linspace(-range, range, steps)
    y = np.linspace(-range, range, steps)
    z = np.array([evaluation(i, j) for j in y for i in x])

    X, Y = np.meshgrid(x, y)
    Z = z.reshape(steps, steps)

    plt.contourf(X, Y, Z, steps)
    plt.colorbar()

def plotGeneration(population, range, evaluation, generation):
    plotFunction(range, evaluation)
    for particle in population.population:
        plt.plot(particle[0], particle[1], 'wo')
    plt.title("Generation " + str(generation))
    return f'{generation}.png'

def main():
    filenames = []
    mins = []
    means = []
    r = 5
    optimization = rosenbrock
    population = EA(20, r, optimization)
    generation = 1

    for i in range(20):
        filename = plotGeneration(population, r, optimization, generation)
        filenames.append(filename)
        plt.savefig(filename)
        plt.close()
        minimum, mean = population.evolve()
        mins.append(minimum)
        means.append(mean)
        generation += 1

    # Create GIF
    with imageio.get_writer('EA.gif', mode='I') as writer:
        for filename in filenames:
            image = imageio.imread(filename)
            writer.append_data(image)

    # Remove files
    for filename in set(filenames):
        os.remove(filename)

    plt.plot(mins)
    plt.xlabel("Minimum evaluation")
    plt.ylabel("Generation")
    plt.title("Minimum evaluations per generation")
    plt.show()

    plt.plot(means)
    plt.xlabel("Average evaluation")
    plt.ylabel("Generation")
    plt.title("Average evaluations per generation")
    plt.show()

if __name__ == "__main__":
    main()