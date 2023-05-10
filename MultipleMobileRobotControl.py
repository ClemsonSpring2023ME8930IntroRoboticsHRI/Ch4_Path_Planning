# -*- coding: utf-8 -*-
"""
Created on Sun May  7 17:34:17 2023

@author: Ryan
Sample example to control multiple mobile robots using A-star
"""

import heapq

# Define the grid size and start/goal points for each robot
GRID_SIZE = (10, 10)
START_POINTS = [(0, 0), (9, 9)]
GOAL_POINTS = [(9, 9), (0, 0)]

# Define the obstacles in the grid
OBSTACLES = [(3, 3), (4, 3), (5, 3), (6, 3)]

# Define the cost of moving between adjacent cells
MOVE_COST = 1

# Define the robot velocity in m/s
ROBOT_VELOCITY = 0.5

# Define the heuristic function for A-star
def heuristic(point, goal):
    return abs(point[0] - goal[0]) + abs(point[1] - goal[1])

# Define the A-star search algorithm
def a_star_search(grid, start, goal):
    # Initialize the open and closed sets
    open_set = []
    closed_set = set()
    heapq.heappush(open_set, (0, start))
    
    # Initialize the g-score and f-score dictionaries
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        # Get the cell with the lowest f-score
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            # Return the path if the goal has been reached
            path = [current]
            while current in g_score:
                current = g_score[current]
                path.append(current)
            return path[::-1]
        
        closed_set.add(current)
        
        # Check the neighbors of the current cell
        for x, y in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            neighbor = current[0] + x, current[1] + y
            tentative_g_score = g_score[current] + MOVE_COST
            
            if 0 <= neighbor[0] < grid.shape[0]:
                if 0 <= neighbor[1] < grid.shape[1]:
                    if grid[neighbor[0]][neighbor[1]] == 0:
                        if neighbor in closed_set:
                            continue
                        if tentative_g_score < g_score.get(neighbor, float('inf')):
                            g_score[neighbor] = tentative_g_score
                            f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    # Return None if no path can be found
    return None

# Define the function to get the combined path for two robots
def get_combined_path(grid, start_points, goal_points):
    paths = []
    for start, goal in zip(start_points, goal_points):
        paths.append(a_star_search(grid, start, goal))
    
    combined_path = []
    max_path_len = max([len(path) for path in paths])
    for i in range(max_path_len):
        for path in paths:
            if i < len(path):
                combined_path.append(path[i])
    return combined_path

# Define the function to calculate the robot velocities
def calculate_velocities(path):
    velocities = []
    for i in range(len(path) - 1):
        distance = ((path[i+1][0] - path[i][0])**2 + (path[i+1][1] - path[i][1])**2)**0.5
        time = distance / ROBOT_VELOCITY
        velocities.append((distance, time)