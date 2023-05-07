# -*- coding: utf-8 -*-
"""
Created on Sun May  7 01:10:05 2023

@author: Ryan
"""

import random
import math
import matplotlib.pyplot as plt

# Define the start and goal points
start = (0, 0)
goal = (9, 9)

# Define the bounds of the graph
x_min, x_max = 0, 10
y_min, y_max = 0, 10

# Define the step size for the algorithm
delta_q = 0.5

# Define the maximum distance for the algorithm
max_dist = 2

# Define the number of iterations for the algorithm
max_iter = 500

# Define the radius of the ball for the algorithm
r = 1

# Define the function to calculate the distance between two points
def dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

# Define the function to check if a point is within the bounds of the graph
def is_valid(point):
    if point[0] < x_min or point[0] > x_max or point[1] < y_min or point[1] > y_max:
        return False
    return True

# Define the function to check if there is an obstacle in the path between two points
def is_obstacle(p1, p2):
    if p1[0] == p2[0]:
        x = p1[0]
        y1, y2 = sorted([p1[1], p2[1]])
        if y1 <= 3 <= y2 and x <= 5:
            return True
    elif p1[1] == p2[1]:
        y = p1[1]
        x1, x2 = sorted([p1[0], p2[0]])
        if x1 <= 5 <= x2 and y <= 3:
            return True
    else:
        x1, y1 = p1
        x2, y2 = p2
        m = (y2 - y1) / (x2 - x1)
        b = y1 - m * x1
        x = (3 - b) / m
        if x1 <= x <= x2 or x2 <= x <= x1:
            return True
    return False

# Define the RRT* algorithm
class RRTStar:
    def __init__(self, start, goal, delta_q, max_dist, max_iter, r):
        self.start = start
        self.goal = goal
        self.delta_q = delta_q
        self.max_dist = max_dist
        self.max_iter = max_iter
        self.r = r
        self.vertices = [start]
        self.edges = []
        self.costs = [0]

    def generate_random_point(self):
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        return (x, y)

    def find_nearest_vertex(self, point):
        distances = [dist(point, v) for v in self.vertices]
        return self.vertices[distances.index(min(distances))]

    def extend_tree(self):
        q_rand = self.generate_random_point()
        q_near = self.find_nearest_vertex(q_rand)
        q_new = q_near
        if not is_obstacle(q_near, q_rand) and dist(q_near, q_rand) <= self.max_dist:
            q_new = q_rand
        elif not is_obstacle(q_near, q_rand):
            q_new = (q_near[0] + self)
