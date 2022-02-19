import copy
import math
import pygame

from Robot import Robot

sev = 35  # SCREEN_EDGE_VACANCY
w = 900
h = 900
line0 = [(sev, sev), (sev, h - sev), (w - sev, h - sev), (w - sev, sev), (sev, sev)]
line1 = [(300, sev), (300, 750)]
line2 = [(600, sev), (600, 300)]
line3 = [(600, 450), (600, h - sev)]
room1 = [line0, line1, line2, line3]
line_star = [(450, 50), (539, 327), (830, 326), (595, 496), (658, 773), (450, 602), (165, 773), (305, 496), (70, 326),
             (360, 327), (450, 50)]
star_room = [line_star]
chosen_room = star_room
initPosition = [400, 600]
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
        self.robot = Robot(chosen_room, initPosition, 60)
        self.screen = pygame.display.set_mode((w, h))

        self.Sur = pygame.Surface
        pygame.display.set_caption("Modular Robot Simulator")
        self.font = pygame.font.SysFont("Pokemon GB.ttf", 50)
        self.sensor_font = pygame.font.SysFont("Pokemon GB.ttf", 20)
        self.running = True
        self.clock = pygame.time.Clock()
        self.dust = []

    def show(self, keys, velocities):
        self.screen.fill(WHITE)
        # Fill in robot environment
        for wall in chosen_room:
            pygame.draw.aalines(self.screen, DARKGRAY, True, wall, 50)
        # Display robot
        rect = pygame.draw.circle(self.screen, PURPLE, (self.robot.x, self.robot.y), 30)
        self.dust.append(rect)
        # Display robot direction
        pygame.draw.line(self.screen, BLACK, (self.robot.x, self.robot.y), (self.robot.frontX, self.robot.frontY), 1)
        # Display robot angle
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
            aa = self.screen.blit(self.sensor_font.render(str(round(sensors[i])), True, BLACK), (x, y))
            angle += math.pi / 6
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

        velocities = self.robot.move(movement, delta_t, chosen_room)

        return movement, velocities

    def stop(self):
        self.running = False
        exit()


def main():
    simulation = Simulation()
    simulation.run()


if __name__ == "__main__":
    main()
