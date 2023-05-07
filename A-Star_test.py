# -*- coding: utf-8 -*-
"""
Created on Sat Apr 15 19:54:18 2023

@author: Ryan
"""
# A-Star's Algorithm in Python
# Source: https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2

import heapq

def a_star(start, goal, graph, h):
    """
    A* algorithm implementation
    :param start: Starting node
    :param goal: Goal node
    :param graph: Graph represented as a dictionary
    :param h: Heuristic function
    :return: Shortest path from start to goal
    """
    open_list = [(0, start)]
    closed_list = []
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = h(start, goal)
    while open_list:
        current_f, current = heapq.heappop(open_list)
        if current == goal:
            path = []
            while current in graph:
                path.append(current)
                current = graph[current]
            path.append(start)
            return path[::-1]
        closed_list.append(current)
        for neighbor in graph[current]:
            if neighbor in closed_list:
                continue
            tentative_g_score = g_score[current] + graph[current][neighbor]
            if tentative_g_score < g_score[neighbor]:
                graph[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + h(neighbor, goal)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))
    return None

# Sample graph
graph = {
    'S': [('B', 1), ('D', 12)],
    'D': [('B', 3), ('L', 4), ('S', 12)],
    'E': [('B', 5), ('G', 1), ('L',10),('A', 12),('F', 5)],
    'L': [('D', 4),('M',2),('E', 10)],
    'M': [('L', 2),('G',7)],
    'G': [('E', 1),('M',7)],
    'F': [('E', 5),('A',16)],
    'A': [('B', 4),('E',12),('F', 16)],
    'B': [('S',1),('E',5), ('A',4)]

}

# Sample heuristic function
def heuristic(node1, node2):
    heuristic_values = {'S': 18, 'D': 14, 'E': 12, 'L': 14, 'M': 8, 'G': 7, 'F': 5, 'A': 3, 'B': 0}
    return heuristic_values[node1]

# Find shortest path
start_node = 'S'
goal_node = 'G'
path = a_star(start_node, goal_node, graph, heuristic)
print(path)

