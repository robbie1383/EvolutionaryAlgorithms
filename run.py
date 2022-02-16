import copy
import math
import numpy as np
import pygame

from Robot import Robot

SEV = 35  # SCREEN_EDGE_VACANCY
WIDTH = 835
HEIGHT = 835

# A list of RGB values for the colours used in the game.
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
DARKGRAY = (125, 130, 138)
PURPLE = (114, 85, 163)
LIGHTPURPLE = (185, 167, 217)


def getColour(pressed):
    if pressed:
        return PURPLE
    else:
        return LIGHTPURPLE


class Simulation:

    def __init__(self):
        pygame.init()
        self.outer_wall = [(SEV, SEV), (SEV, WIDTH - SEV), (WIDTH - SEV, HEIGHT - SEV), (HEIGHT - SEV, SEV), (SEV, SEV)]
        self.inner_wall = [(300, 400), (400, 500), (500, 400), (400, 300), (300, 400)]
        self.robot = Robot(self.outer_wall, self.inner_wall, 100)
        self.screen = pygame.display.set_mode((WIDTH + 350, HEIGHT))
        pygame.display.set_caption("Modular Robot Simulator")
        self.font = pygame.font.SysFont("Pokemon GB.ttf", 50)
        self.sensor_font = pygame.font.SysFont("Pokemon GB.ttf", 20)
        self.running = True
        self.clock = pygame.time.Clock()

    def show(self, keys, velocities):
        self.screen.fill(WHITE)

        # Fill in robot environment
        pygame.draw.aalines(self.screen, DARKGRAY, True, self.outer_wall, 50)
        pygame.draw.aalines(self.screen, DARKGRAY, True, self.inner_wall, 50)
        pygame.draw.circle(self.screen, PURPLE, (self.robot.x, self.robot.y), 50)
        pygame.draw.line(self.screen, BLACK, (self.robot.x, self.robot.y), (self.robot.frontX, self.robot.frontY), 1)

        # display sensors
        angle = copy.copy(self.robot.theta)
        sensors = self.robot.sensors
        for i in range(12):
            x, y = self.robot.rotate(angle, self.robot.radius + 12)
            x = x - 10
            y = y - 8
            self.screen.blit(self.sensor_font.render(str(round(sensors[i])), True, BLACK), (x, y))
            angle += math.pi / 6

        # Fill in keys
        pygame.draw.rect(self.screen, getColour(keys[0]), pygame.Rect(WIDTH + 50, HEIGHT / 2 - 100, 50, 50))
        self.screen.blit(self.font.render("w", 105, WHITE), (WIDTH + 60, HEIGHT / 2 - 95))
        pygame.draw.rect(self.screen, getColour(keys[1]), pygame.Rect(WIDTH + 50, HEIGHT / 2, 50, 50))
        self.screen.blit(self.font.render("s", 105, WHITE), (WIDTH + 65, HEIGHT / 2 + 5))
        pygame.draw.rect(self.screen, getColour(keys[5]), pygame.Rect(WIDTH + 150, HEIGHT / 2 - 100, 50, 50))
        self.screen.blit(self.font.render("t", 105, WHITE), (WIDTH + 170, HEIGHT / 2 - 90))
        pygame.draw.rect(self.screen, getColour(keys[6]), pygame.Rect(WIDTH + 150, HEIGHT / 2, 50, 50))
        self.screen.blit(self.font.render("g", 105, WHITE), (WIDTH + 165, HEIGHT / 2 + 5))
        pygame.draw.rect(self.screen, getColour(keys[2]), pygame.Rect(WIDTH + 250, HEIGHT / 2 - 100, 50, 50))
        self.screen.blit(self.font.render("o", 105, WHITE), (WIDTH + 265, HEIGHT / 2 - 95))
        pygame.draw.rect(self.screen, getColour(keys[3]), pygame.Rect(WIDTH + 250, HEIGHT / 2, 50, 50))
        self.screen.blit(self.font.render("l", 105, WHITE), (WIDTH + 270, HEIGHT / 2 + 10))
        pygame.draw.rect(self.screen, getColour(keys[4]), pygame.Rect(WIDTH + 150, HEIGHT / 2 + 100, 50, 50))
        self.screen.blit(self.font.render("x", 105, WHITE), (WIDTH + 165, HEIGHT / 2 + 105))

        # Display velocities
        left = "V left = " + str(np.round(velocities[0],2))
        self.screen.blit(self.font.render(left, 105, BLACK), (WIDTH + 50, HEIGHT / 2 - 305))
        right = "V right = " + str(np.round(velocities[1],2))
        self.screen.blit(self.font.render(right, 105, BLACK), (WIDTH + 50, HEIGHT / 2 - 255))
        theta = "θ = " + str(np.round(velocities[2] % 360, 2)) + "°"
        self.screen.blit(self.font.render(theta, 105, BLACK), (WIDTH + 50, HEIGHT / 2 - 205))
        pygame.display.flip()

    def run(self):
        delta_t = 0.01
        while self.running:
            self.clock.tick(50)
            keys, velocities = self.update(delta_t)
            self.show(keys, velocities)
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

        velocities = self.robot.move(movement, delta_t, self.outer_wall, self.inner_wall)

        return movement, velocities

    def stop(self):
        self.running = False
        exit()


def main():
    simulation = Simulation()
    simulation.run()


if __name__ == "__main__":
    main()
