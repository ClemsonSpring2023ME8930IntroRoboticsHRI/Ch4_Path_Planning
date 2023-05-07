# -*- coding: utf-8 -*-
"""
Created on Sat May  6 18:36:43 2023

@author: Ryan

The prm function takes in an array of points, k which is the size of the area, num_samples which is the number of random samples to add to the graph, and radius which is the maximum distance between two nodes for them to be connected by an edge.

The function creates a Graph object and adds nodes to it for each point in the points array. It then connects these nodes if they are within the given radius. The function then adds num_samples random nodes to the graph and connects them to nearby nodes. The resulting Graph object is returned.

In the example usage, we create an array of points and run the prm function with k=5, num_samples=10, and radius=1. The resulting graph is printed along with the number of nodes and edges in the graph. Finally, the graph is plotted using matplotlib.
"""

import numpy as np
import matplotlib.pyplot as plt

class Graph:
    def __init__(self):
        self.nodes = []
        self.edges = []

    def add_node(self, node):
        self.nodes.append(node)

    def add_edge(self, edge):
        self.edges.append(edge)

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.edges = []

class Edge:
    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.length = np.sqrt((end.x - start.x)**2 + (end.y - start.y)**2)

def prm(points, k, num_samples, radius):
    graph = Graph()

    # Add nodes to the graph
    for p in points:
        graph.add_node(Node(p[0], p[1]))

    # Connect nodes in the graph
    for i, node1 in enumerate(graph.nodes):
        for j, node2 in enumerate(graph.nodes):
            if i != j:
                edge = Edge(node1, node2)
                if edge.length <= radius:
                    node1.edges.append(edge)
                    node2.edges.append(edge)
                    graph.add_edge(edge)

    # Add random nodes to the graph
    for i in range(num_samples):
        p = np.random.uniform(low=[0, 0], high=[k, k], size=(1, 2))[0]
        node = Node(p[0], p[1])
        graph.add_node(node)

        # Connect the new node to nearby nodes in the graph
        for n in graph.nodes:
            if n == node:
                continue
            edge = Edge(n, node)
            if edge.length <= radius:
                n.edges.append(edge)
                node.edges.append(edge)
                graph.add_edge(edge)

    return graph

# Example usage
points = np.array([[4,1], [3,3], [6,3], [7,1], [9,2], [9,4], [6,5], [3,5], [2,5], [1,3], [2,2], [10,4]])
graph = prm(points, k=5, num_samples=20, radius=1)
print(f"Number of nodes: {len(graph.nodes)}")
print(f"Number of edges: {len(graph.edges)}")
print(f"Number of nodes: {len(graph.nodes)}")
print(f"Number of edges: {len(graph.edges)}")

# Plot the graph
for node in graph.nodes:
    plt.plot(node.x, node.y, 'bo')
    for edge in node.edges:
        plt.plot([edge.start.x, edge.end.x], [edge.start.y, edge.end.y], 'k-')
plt.axis('equal')
plt.show()
