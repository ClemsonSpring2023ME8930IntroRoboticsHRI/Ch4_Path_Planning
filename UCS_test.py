# -*- coding: utf-8 -*-
"""
Created on Sat Apr 15 19:46:13 2023

@author: Ryan
"""

# UCS's Algorithm in Python
# Source: https://www.geeksforgeeks.org/uniform-cost-search-dijkstra-for-large-graphs/
# returns the minimum cost in a vector( if there are multiple goal states)
import heapq

def UCS(graph, start, goal):
    # initialize frontier with start node
    frontier = [(0, start, [])]
    # initialize explored set
    explored = set()
    
    # loop until frontier is empty
    while frontier:
        # get the node with the lowest cost from the frontier
        (cost, current, path) = heapq.heappop(frontier)
        
        # check if the current node is the goal
        if current == goal:
            return path + [current]
        
        # add the current node to the explored set
        explored.add(current)
        
        # expand the current node's neighbors
        for neighbor, neighbor_cost in graph[current]:
            # check if neighbor has not been explored
            if neighbor not in explored:
                # add the neighbor to the frontier with the new cost and path
                new_cost = cost + neighbor_cost
                new_path = path + [current]
                heapq.heappush(frontier, (new_cost, neighbor, new_path))
    
    # return None if no path is found
    return None


# Example usage:
graph = {
    'S': [('B', 2), ('D', 5)],
    'D': [('G', 6), ('E', 2), ('B', 5), ('S', 5)],
    'E': [('G', 7), ('F', 3), ('A',3),('D', 2)],
    'F': [('A', 6),('G',3),('E', 3)],
    'A': [('B', 4),('E',3),('F', 6)],
    'B': [('G', 1),('D',5),('S', 2),('A',4)],
    'G': [('B',1),('D',6), ('E',7), ('F',3)]

}

start_node = 'S'
end_node = 'G'

print(UCS(graph, start_node, end_node)) # Output: S,B,G
