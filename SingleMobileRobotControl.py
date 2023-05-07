# -*- coding: utf-8 -*-
"""
Created on Sun May  7 17:38:03 2023

@author: Ryan
"""

import sim
import math

class MobileRobot:
    def __init__(self, client_id, name):
        self.client_id = client_id
        self.name = name
        # Get handles for the robot's components
        _, self.robot = sim.simxGetObjectHandle(self.client_id, self.name, sim.simx_opmode_blocking)
        _, self.left_motor = sim.simxGetObjectHandle(self.client_id, self.name + '_leftMotor', sim.simx_opmode_blocking)
        _, self.right_motor = sim.simxGetObjectHandle(self.client_id, self.name + '_rightMotor', sim.simx_opmode_blocking)
        _, self.front_sensor = sim.simxGetObjectHandle(self.client_id, self.name + '_frontSensor', sim.simx_opmode_blocking)

    def set_left_motor_speed(self, speed):
        sim.simxSetJointTargetVelocity(self.client_id, self.left_motor, speed, sim.simx_opmode_oneshot)

    def set_right_motor_speed(self, speed):
        sim.simxSetJointTargetVelocity(self.client_id, self.right_motor, speed, sim.simx_opmode_oneshot)

    def read_front_sensor(self):
        _, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(self.client_id, self.front_sensor, sim.simx_opmode_blocking)
        if detection_state:
            return math.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
        else:
            return None

    def move_forward(self, speed):
        self.set_left_motor_speed(speed)
        self.set_right_motor_speed(speed)

    def turn_left(self, speed):
        self.set_left_motor_speed(-speed)
        self.set_right_motor_speed(speed)

    def turn_right(self, speed):
        self.set_left_motor_speed(speed)
        self.set_right_motor_speed(-speed)

    def stop(self):
        self.set_left_motor_speed(0)
        self.set_right_motor_speed(0)

def a_star_search(start_pos, goal_pos, obstacle_list):
    # Define the heuristic function for A* search
    def euclidean_distance(pos1, pos2):
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)

    # Define the cost function for A* search
    def cost_function(pos1, pos2):
        return euclidean_distance(pos1, pos2)

    # Define the node class for A* search
    class Node:
        def __init__(self, position, parent=None):
            self.position = position
            self.parent = parent
            self.g_cost = 0
            self.h_cost = euclidean_distance(position, goal_pos)
            self.f_cost = self.g_cost + self.h_cost

        def update_costs(self, parent, new_g_cost):
            self.parent = parent
            self.g_cost = new_g_cost
            self.f_cost = self.g_cost + self.h_cost

    # Initialize the open and closed sets for A* search
    start_node = Node(start_pos)
    open_set = [start_node]
    closed_set = []

    # A* search algorithm loop
    while open_set:
        # Find the node in the open set with the lowest f_cost
        current_node = min(open_set, key=lambda node: node.f_cost)

        # If the current node is the goal node, return the path
        if current_node.position == goal_pos:
            path = []
            while current_node:
                path.append(current_node.position)
                
               
