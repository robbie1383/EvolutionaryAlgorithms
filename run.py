import copy
import math
import pygame
from matplotlib import pyplot as plt
import re
from Robot import Robot
from RobotNN import RobotEA, RobotNN

sev = 35  # SCREEN_EDGE_VACANCY
w = 900
h = 900
line0 = [(sev, sev), (sev, h - sev), (w - sev, h - sev), (w - sev, sev), (sev, sev)]
line1 = [(300, sev), (300, 750), (340, 750), (340, sev)]
line2 = [(600, sev), (600, 300), (640, 300), (640, sev)]
line3 = [(640, h - sev), (640, 450), (600, 450), (600, h - sev)]
room1 = [line0, line1, line2, line3]
room2 = [line0]
line_star = [(450, 50), (539, 327), (830, 326), (595, 496), (658, 773), (450, 602), (165, 773), (305, 496), (70, 326),
             (360, 327), (450, 50)]
star_room = [line_star]
chosen_room = room2
initPosition = [400, 600]
# A list of RGB values for the colours used in the game.
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
DARKGRAY = (125, 130, 138)
PURPLE = (114, 85, 163)
LIGHTPURPLE = (185, 167, 217)
orange = (255, 100, 10)
yellow = (255, 255, 0)


class Simulation:

    def __init__(self):
        pygame.init()
        self.robot = Robot(chosen_room, initPosition, 60)
        self.screen = pygame.display.set_mode((w, h))
        self.Sur = pygame.Surface
        pygame.display.set_caption("Modular Robot Simulator")
        self.font = pygame.font.SysFont("Pokemon GB.ttf", 50)
        self.sensor_font = pygame.font.SysFont("Pokemon GB.ttf", 20)
        self.running = True
        self.clock = pygame.time.Clock()
        self.clean_area = []

    def show(self, velocities):
        self.screen.fill(WHITE)
        for p in self.clean_area:
            pygame.draw.circle(self.screen, yellow, (p[0], p[1]), 30)
        # Fill in robot environment
        for wall in chosen_room:
            pygame.draw.aalines(self.screen, DARKGRAY, True, wall, 50)
        # Display robot
        pygame.draw.circle(self.screen, PURPLE, (self.robot.x, self.robot.y), 30)

        pygame.draw.line(self.screen, BLACK, (self.robot.x, self.robot.y), (self.robot.frontX, self.robot.frontY), 1)
        angle_x = (self.robot.x + self.robot.frontX) / 2 - 5
        angle_y = (self.robot.y + self.robot.frontY) / 2 - 5
        self.screen.blit(self.sensor_font.render(str(round(velocities[2] % 360)), True, BLACK), (angle_x, angle_y))
        # Display speed
        left_x, left_y = self.robot.rotate(self.robot.theta + math.pi / 2, self.robot.radius - 10)
        self.screen.blit(self.sensor_font.render(str(round(velocities[0])), True, BLACK), (left_x, left_y))
        right_x, right_y = self.robot.rotate(self.robot.theta - math.pi / 2, self.robot.radius - 25)
        self.screen.blit(self.sensor_font.render(str(round(velocities[1])), True, BLACK), (right_x, right_y))
        self.screen.blit(self.sensor_font.render(velocities[4], True, BLACK), (5, 5))

        # display  12 sensors
        angle = copy.copy(self.robot.theta)
        sensors = self.robot.sensors
        for i in range(12):
            x, y = self.robot.rotate(angle, self.robot.radius + 15)
            x = x - 10
            y = y - 8
            self.screen.blit(self.sensor_font.render(str(round(sensors[i])), True, BLACK), (x, y))
            angle += math.pi / 6

        pygame.display.flip()

    def run(self, nn, delta_t):
        while self.running:
            self.clock.tick(50)
            velocities = self.updatefromNN(delta_t, nn)
            self.show(velocities)
            self.clean_area.append([self.robot.x, self.robot.y])
            delta_t = velocities[3]

    def update(self, delta_t):
        # Quit the simulation
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.stop()
                pygame.quit()

        # Get pressed keys and update robot position
        keys = pygame.key.get_pressed()
        movement = [keys[pygame.K_w], keys[pygame.K_s], keys[pygame.K_o], keys[pygame.K_l], keys[pygame.K_x],
                    keys[pygame.K_t], keys[pygame.K_g]]

        velocities = self.robot.move(movement, delta_t, chosen_room)
        return velocities

    def updatefromNN(self, delta_t, nn):
        # Quit the simulation
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.stop()
                pygame.quit()
        [Vl, Vr] = nn.activations(self.robot.sensors)[1]
        _, velocities = self.robot.moveFromVelocities(Vr, Vl, delta_t, chosen_room)
        # print(velocities)
        return velocities

    def stop(self):
        self.running = False
        exit()


def plot(mins, means):
    plt.plot(mins)
    plt.ylabel("Minimum evaluation")
    plt.xlabel("Generation")
    plt.title("Minimum evaluations per generation")
    plt.show()

    plt.plot(means)
    plt.ylabel("Average evaluation")
    plt.xlabel("Generation")
    plt.title("Average evaluations per generation")
    plt.show()


def train(iterations, theta_t):
    networks = open("networks.txt", "w")
    ea = RobotEA(chosen_room, theta_t, initPosition)
    mins = []
    means = []
    print("Started learning.")
    for generation in range(iterations + 1):
        print(generation, " / ", iterations)
        min, avg, best = ea.evolve()
        mins.append(min)
        means.append(avg)
        print("Average evaluation after learning:", avg)
        if generation % 10 == 0:
            print("Added generation ", generation, "to the file.")
            networks.write("Generation " + str(generation) + "\n")
            networks.write(str(best) + "\n")
    plot(mins, means)
    networks.close()


def display_result(generation, delta_t):
    networks = open("networks.txt", "r")
    text = networks.read()
    numbers = re.sub("[^0123456789\.]", " ", text)
    numbers = [float(i) for i in numbers.split(" ") if len(i) > 0]
    start = numbers.index(generation)
    net = []
    layer1 = []
    for i in range(4):
        layer1.append([numbers[i] for i in range(start + 1, start + 17)])
        start += 16
    net.append(layer1)
    layer2 = []
    for i in range(2):
        layer2.append([numbers[i] for i in range(start + 1, start + 5)])
        start += 4
    net.append(layer2)
    nn = RobotNN(net)
    print(len(nn.network))
    sim = Simulation()
    sim.run(nn, delta_t)
    networks.close()


def main():
    delta_t = 0.1
    generations = 50
    # train(generations, delta_t)
    display_result(generations, delta_t)


if __name__ == "__main__":
    main()
