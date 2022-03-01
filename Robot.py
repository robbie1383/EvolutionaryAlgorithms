import math
import random
from copy import copy
import numpy as np
from pygame.math import Vector2


# calculate the slope of the line
def slope(wall):
    x1, y1 = wall[0]
    x2, y2 = wall[1]
    if x1 == x2:
        return 90
    elif y1 == y2:
        return 0
    else:
        return math.atan((y2 - y1) / (x2 - x1)) * 180 / math.pi


def right_intersection(point, wall_point1, wall_point2, sensor_start, sensor_end):
    x, y = point
    right = (x - wall_point1[0]) * (x - wall_point2[0]) <= 0 and \
            (y - wall_point1[1]) * (y - wall_point2[1]) <= 0 and \
            (x - sensor_start[0]) * (sensor_end[0] - sensor_start[0]) >= 0 and \
            (y - sensor_start[1]) * (sensor_end[1] - sensor_start[1]) >= 0
    return right


def get_intersection(point1, point2, point3, point4):
    new_x, new_y = 10000, 10000
    a1 = point2[1] - point1[1]
    b1 = point1[0] - point2[0]
    c1 = a1 * point1[0] + b1 * point1[1]
    # Sensor line
    a2 = point4[1] - point3[1]
    b2 = point3[0] - point4[0]
    c2 = a2 * point3[0] + b2 * point3[1]
    determinant = a1 * b2 - a2 * b1
    if abs(determinant) > 0.00000001:  # if there is an intersectioin
        new_x = -(b1 * c2 - b2 * c1) / determinant  # intersection coordinate
        new_y = (a1 * c2 - a2 * c1) / determinant
        if abs(round(new_x) - new_x) < 0.00000001:
            new_x = round(new_x)
        if abs(round(new_y) - new_y) < 0.00000001:
            new_y = round(new_y)
    return new_x, new_y


class Robot:

    def __init__(self, walls, initPosition, size):
        self.radius = int(size / 2)
        self.sensor_limit = self.radius
        self.x, self.y = self.initPosition(initPosition)
        self.frontX = self.x + self.radius
        self.frontY = self.y
        self.Vl = 0
        self.Vr = 0
        self.theta = 0
        self.speed = 3
        self.sensors, _ = self.distanceToSensors(walls)
        self.stop = False
        # Cover as much area as possible
        self.dust = set()
        # For fitness: Avoids robots that do not move enough
        self.invalid_move = 0
        self.clean_dust()
        # For fitness:Motivates robots to move closer to walls
        self.wall_close_dust = set()
        # For fitness collision free
        self.collision = 0
        self.move_counter = 0

    def initPosition(self, position):
        x = random.randint(position[0], position[0])
        y = random.randint(position[1], position[1])
        return x, y

    def moveFromVelocities(self, Vr, Vl, delta_t, walls):
        self.Vr = Vr
        self.Vl = Vl
        test = "Safe"
        self.move_counter = self.move_counter + 1
        # If it's moving
        if self.Vr != 0 or self.Vl != 0:
            # Make sure not to get a division by zero when velocities are the same
            if self.Vr == self.Vl:
                next_x = self.x + ((self.Vl + self.Vr) / 2) * np.cos(self.theta) * delta_t
                next_y = self.y - ((self.Vl + self.Vr) / 2) * np.sin(self.theta) * delta_t
                new_theta = self.theta + (self.Vr - self.Vl) / (2 * self.radius) * delta_t
            else:
                R = self.radius * (self.Vl + self.Vr) / (self.Vr - self.Vl)
                w = (self.Vr - self.Vl) / (self.radius * 2)
                # Compute ICC
                ICC = [self.x - R * np.sin(-self.theta), self.y + R * np.cos(self.theta)]
                result = np.transpose(np.matmul(
                    np.array([[np.cos(w * delta_t), -np.sin(w * delta_t), 0],
                              [np.sin(w * delta_t), np.cos(w * delta_t), 0],
                              [0, 0, 1]]),
                    np.transpose(np.array([self.x - ICC[0], self.y - ICC[1], self.theta]))) + np.array(
                    [ICC[0], ICC[1], w * delta_t])).transpose()
                next_x, next_y, new_theta = result[0], result[1], result[2]
            # update  sensors
            self.sensors, walls = self.distanceToSensors(walls)
            # handle collision
            move_distance = math.sqrt((next_x - self.x) ** 2 + (next_y - self.y) ** 2)
            if move_distance <= 1 or move_distance > self.radius / 2:
                self.invalid_move = self.invalid_move + 1
            else:
                collision = self.detectCollision(next_x, next_y, walls)
                if collision:
                    test = "Danger!"
                    self.collision = self.collision + 1
                else:
                    # Fitness, Avoids robots that do not move enough: move distance<0.2
                    self.x = next_x
                    self.y = next_y
                # Fitness: Cover as much area as possible: simulate dust, use removed dust as fitness
                dusts = self.clean_dust()
                self.dust.update(dusts)
                if min(self.sensors) < 6:
                    self.wall_close_dust.update(dusts)

            # Transfer results from the ICC computation
            self.theta = new_theta
            self.frontX, self.frontY = self.rotate(self.theta, self.radius)
        fit = self.fitness()
        return [fit, self.x, self.y], [self.Vl, self.Vr, np.round(np.degrees(self.theta), 2), delta_t, test]

    def move(self, movement, delta_t, walls):
        # Check keys for movement
        # movement = [w, s, o, l, x, t, g]
        test = "Safe"
        if movement[0] == 1 or movement[5] == 1:
            self.Vl += self.speed
        if movement[1] == 1 or movement[6] == 1:
            self.Vl -= self.speed
        if movement[2] == 1 or movement[5] == 1:
            self.Vr += self.speed
        if movement[3] == 1 or movement[6] == 1:
            self.Vr -= self.speed
        if movement[4] == 1:
            self.Vl = 0
            self.Vr = 0

        # If it's moving
        if self.Vr != 0 or self.Vl != 0:
            # Make sure not to get a division by zero when velocities are the same
            if self.Vr == self.Vl:
                next_x = self.x + ((self.Vl + self.Vr) / 2) * np.cos(self.theta) * delta_t
                next_y = self.y - ((self.Vl + self.Vr) / 2) * np.sin(self.theta) * delta_t
                new_theta = self.theta + (self.Vr - self.Vl) / (2 * self.radius) * delta_t
            else:
                R = self.radius * (self.Vl + self.Vr) / (self.Vr - self.Vl)
                w = (self.Vr - self.Vl) / (self.radius * 2)
                # Compute ICC
                ICC = [self.x - R * np.sin(-self.theta), self.y + R * np.cos(self.theta)]
                result = np.transpose(np.matmul(
                    np.array([[np.cos(w * delta_t), -np.sin(w * delta_t), 0],
                              [np.sin(w * delta_t), np.cos(w * delta_t), 0],
                              [0, 0, 1]]),
                    np.transpose(np.array([self.x - ICC[0], self.y - ICC[1], self.theta]))) + np.array(
                    [ICC[0], ICC[1], w * delta_t])).transpose()
                next_x, next_y, new_theta = result[0], result[1], result[2]
            # update  sensors
            self.sensors, walls = self.distanceToSensors(walls)
            # handle collision
            collision = self.detectCollision(next_x, next_y, walls)
            if collision:
                test = "Danger!"
                self.collision = self.collision + 1
            # Avoids robots that do not move enough: move distance<1
            if math.sqrt((next_x - self.x) ** 2 + (next_y - self.y) ** 2) <= 1:
                self.invalid_move = self.invalid_move + 1
            # Transfer results from the ICC computation
            self.x = next_x
            self.y = next_y
            dusts = self.clean_dust()
            self.dust.update(dusts)
            if min(self.sensors) < 3:
                self.wall_close_dust.update(dusts)
            self.theta = new_theta
            self.frontX, self.frontY = self.rotate(self.theta, self.radius)

        return self.Vl, self.Vr, np.round(np.degrees(self.theta), 2), delta_t, test

    def distanceToSensors(self, all_walls):
        dist = []
        detected_walls = []
        angle = copy(self.theta)
        for i in range(12):
            min_dist_out_wall, wall_out = self.distance(all_walls, angle)
            if min_dist_out_wall >= self.sensor_limit:
                min_dist_out_wall = self.sensor_limit
            dist.append(min_dist_out_wall)
            detected_walls.append(wall_out)
            angle += math.pi / 6
        return dist, detected_walls

    def rotate(self, angle, r):
        # Rotate the robot at a certain angle from the x-axis
        front_x = self.x + np.cos(angle) * r
        front_y = self.y + np.sin(-angle) * r
        return front_x, front_y

    def distance(self, walls, angle):
        dist = []
        select_wall = []
        for wall in walls:
            for i in range(len(wall) - 1):
                # Out wall line
                point1 = wall[i]
                point2 = wall[i + 1]
                # Sensor line
                point3 = [self.x, self.y]
                point4 = Vector2(self.rotate(angle, self.radius))
                new_x, new_y = get_intersection(point1, point2, point3, point4)
                if right_intersection((new_x, new_y), point1, point2, point3, point4):
                    dist.append(math.sqrt((new_x - point4[0]) ** 2 + (new_y - point4[1]) ** 2))
                    select_wall.append([point1, point2])
        walls = []
        if len(dist):
            min_dist_out_wall = min(dist)  # CLoser wall to sensor
            wall_index = dist.index(min_dist_out_wall)
            walls = select_wall[wall_index]
        else:
            min_dist_out_wall = 1500
        return min_dist_out_wall, walls

    def robot_and_wall(self, move_x, move_y, wall, closest_sensor):
        collision = False
        towards_wall = False
        theta_wall = slope(wall)
        theta_move = slope([[self.x, self.y], [move_x, move_y]])
        theta_wall_move = theta_wall - theta_move
        move_distance = ((move_x - self.x) ** 2 + (move_y - self.y) ** 2) ** 0.5
        distance_vertical = abs(move_distance * np.sin(theta_wall_move * math.pi / 180))
        inters_x, inters_y = get_intersection((self.x, self.y), (move_x, move_y), wall[0], wall[1])
        if (move_x - inters_x) * (move_x - self.x) <= 0 and (move_y - inters_y) * (move_y - self.y) <= 0:
            towards_wall = True
        robot_to_wall_point = min(((move_x - wall[0][0]) ** 2 + (move_y - wall[0][1]) ** 2) ** 0.5,
                                  ((move_x - wall[1][0]) ** 2 + (move_y - wall[1][1]) ** 2) ** 0.5)
        if robot_to_wall_point < self.radius:
            collision = True
        else:
            if towards_wall and distance_vertical + 3 > closest_sensor:
                collision = True
        return collision

    def detectCollision(self, move_x, move_y, walls):
        # get the closest distance
        sensors_copy = self.sensors.copy()
        sensors_copy.sort()
        collision_dis = sensors_copy[0]
        second_collision_dis = sensors_copy[1]
        # get the closest wall
        wall_index = self.sensors.index(collision_dis)
        second_wall_index = self.sensors.index(second_collision_dis)
        first_wall = walls[wall_index]
        second_wall = walls[second_wall_index]
        for i in range(len(walls) - 1):
            if first_wall == second_wall:
                second_collision_dis = sensors_copy[i + 1]
                second_wall_index = self.sensors.index(second_collision_dis)
                second_wall = walls[second_wall_index]
            else:
                break
        collision1 = self.robot_and_wall(move_x, move_y, first_wall, collision_dis)
        collision2 = self.robot_and_wall(move_x, move_y, second_wall, second_collision_dis)
        collision = collision1 or collision2
        return collision

    def clean_dust(self):
        dusts = set()
        for x in range(int(self.x) - self.radius - 1, int(self.x) + self.radius + 1):
            for y in range(int(self.y) - self.radius - 1, int(self.y) + self.radius + 1):
                # Calculate the distance from the center of grid to the robot center
                if math.sqrt((x + 0.5 - self.x) ** 2 + (y + 0.5 - self.y) ** 2) <= self.radius:
                    dusts.add(int(y) * 900 + int(x))
        return dusts

    def fitness(self):
        return len(self.dust) - self.collision + 100 * len(self.wall_close_dust) - self.invalid_move
