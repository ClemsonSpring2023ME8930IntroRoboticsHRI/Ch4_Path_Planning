# -*- coding: utf-8 -*-
"""
Created on Sat Apr 15 19:41:15 2023

@author: Ryan
"""

# Dijkstra's Algorithm in Python
# Source: https://www.programiz.com/dsa/dijkstra-algorithm

import heapq

def dijkstra(graph, start, end):
    """
    Dijkstra search algorithm implementation for a weighted graph
    :param graph: dictionary containing nodes as keys and a list of tuples as values
                  each tuple contains the node and its weight
    :param start: starting node
    :param end: target node
    :return: shortest path taken from start to end, as a list of nodes
    """
    distances = {node: float('inf') for node in graph}  # initialize all distances to infinity
    distances[start] = 0  # starting node distance is 0
    priority_queue = [(0, start)]  # tuple with distance and node
    visited = set()  # set to keep track of visited nodes
    path = {}  # dictionary to keep track of shortest path taken
    while priority_queue:
        (current_distance, current_node) = heapq.heappop(priority_queue)  # remove node with smallest distance
        visited.add(current_node)
        for neighbor, weight in graph[current_node]:
            if neighbor not in visited:
                new_distance = distances[current_node] + weight
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    heapq.heappush(priority_queue, (new_distance, neighbor))
                    path[neighbor] = current_node
        if current_node == end:
            break
    shortest_path = [end]
    node = end
    while node != start:
        shortest_path.append(path[node])
        node = path[node]
    shortest_path.reverse()
    return shortest_path
graph = {
    'C': [('F', 2), ('S', 1), ('E',7)],
    'S': [('E', 3), ('C', 1)],
    'E': [('G', 3), ('C', 7), ('B',1)],
    'B': [('F', 3),('E',1)],
    'F': [('G', 7),('B',3)],
    'G': [('F',7),('E',3)]

}

start_node = 'S'
end_node = 'G'

shortest_path = dijkstra(graph, start_node, end_node)

print("Shortest path from", start_node, "to", end_node, ":", shortest_path) # Output: S,E,G
