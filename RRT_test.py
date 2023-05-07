# -*- coding: utf-8 -*-
"""
Created on Sat May  6 23:06:50 2023

@author: Ryan
"""

import math
import random
import matplotlib.pyplot as plt

import numpy as np
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class Graph:
    def __init__(self):
        self.nodes = []
        self.edges = {}

    def add_node(self, node):
        self.nodes.append(node)
        self.edges[node] = []

    def add_edge(self, node1, node2):
        self.edges[node1].append(node2)
        self.edges[node2].append(node1)


class rrt_algorithm:
    def __init__(self, start, goal, obstacle_list, expand_dist=0.5, max_iter=1000):
        self.start = start
        self.goal = goal
        self.obstacle_list = obstacle_list
        self.expand_dist = expand_dist
        self.max_iter = max_iter
        self.graph = Graph()

    def planning(self):
        self.graph.add_node(self.start)

        for i in range(self.max_iter):
            rnd_node = self.generate_random_node()
            nearest_node = self.find_nearest_node(rnd_node)
            new_node = self.steer(nearest_node, rnd_node, self.expand_dist)

            if self.check_collision(new_node):
                continue

            self.graph.add_node(new_node)
            self.graph.add_edge(nearest_node, new_node)

            if self.calc_dist_to_goal(new_node) <= self.expand_dist:
                final_node = self.steer(new_node, self.goal, self.expand_dist)
                if not self.check_collision(final_node):
                    self.graph.add_node(final_node)
                    self.graph.add_edge(new_node, final_node)
                    return self.find_path()

        return None

    def generate_random_node(self):
        if random.randint(0, 100) > 5:
            return Node(random.uniform(0, 10), random.uniform(0, 10))
        else:
            return self.goal

    def find_nearest_node(self, rnd_node):
        nearest_node = self.graph.nodes[0]
        min_dist = math.sqrt((rnd_node.x - nearest_node.x)**2 + (rnd_node.y - nearest_node.y)**2)
        for node in self.graph.nodes[1:]:
            dist = math.sqrt((rnd_node.x - node.x)**2 + (rnd_node.y - node.y)**2)
            if dist < min_dist:
                nearest_node = node
                min_dist = dist
        return nearest_node

    def steer(self, from_node, to_node, extend_length=float("inf")):
        dist = math.sqrt((from_node.x - to_node.x)**2 + (from_node.y - to_node.y)**2)
        if dist < extend_length:
            return to_node
        else:
            theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
            return Node(from_node.x + extend_length * math.cos(theta),
                        from_node.y + extend_length * math.sin(theta))

    def check_collision(self, node):
        for (ox, oy, size) in self.obstacle_list:
            if math.sqrt((node.x - ox)**2 + (node.y - oy)**2) <= size:
                return True
        return False

    def calc_dist_to_goal(self, node):
        return math.sqrt((node.x - self.goal.x)**2 + (node.y - self.goal.y)**2)

    def find_path(self):
        path = [self.goal]
        node = self.graph.nodes[-1]
        while node.parent is not None:
            path =  [self.goal]
            return path
