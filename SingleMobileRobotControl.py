# -*- coding: utf-8 -*-
"""
Created on Sun May  7 17:38:03 2023

@author: Ryan
The program is able to control a single mobile robot to desired goal position
"""

import sim
import math
import time
from zmqRemoteApi import RemoteAPIClient
#import all other packages

client = RemoteAPIClient()
sim = client.getObject('sim')
L=0.5
R=0.5

"connecting to coppeliasim"

print ('Program started')

"just in case, close all open connections"

sim.simxFinish(-1) 

"Connect to CoppeliaSim"

clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) 
if clientID!=-1:
robot = MobileRobot(client_id, 'Pioneer_p3dx')
    print ('Connected to remote API server') 
    
"getting all the handles on the objects in the scene-> whatever objects you need for your algorithm"
"remember that you can extract all sorts of information from the object i.e. rotation matrix, position, euler angles, velocity"


errorCode, right_motor=sim.simxGetObjectHandle(clientID,'rightMotor',sim.simx_opmode_blocking)
errorCode, left_motor=sim.simxGetObjectHandle(clientID,'leftMotor',sim.simx_opmode_blocking)
errorCode, UGV=sim.simxGetObjectHandle(clientID,'PioneerP3DX',sim.simx_opmode_blocking)

"control loop-> run simulation for multiple time steps if that is necessary for your work, for students doing inverse kinematic algorithms, you can also run this as a node"

t=time.time()

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
client_id = sim.simxStart('127.0.0.1',19999,True,True,5000,5)

if client_id!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=sim.simxGetObjects(client_id,sim.sim_handle_all,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    time.sleep(2)

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    startTime=time.time()
    sim.simxGetIntegerParameter(client_id,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) # Initialize streaming
    while time.time()-startTime < 5:
        returnCode,data=sim.simxGetIntegerParameter(client_id,sim.sim_intparam_mouse_x,sim.simx_opmode_buffer) # Try to retrieve the streamed data
        if returnCode==sim.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
            print ('Mouse position x: ',data) # Mouse position x is actualized when the cursor is over CoppeliaSim's window
        time.sleep(0.005)

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(client_id,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(client_id)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(client_id)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

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
                current = current_node.position
                path.append(current_node.position)
            return path[]
           closed_set.add(current)
            
        
   # Return None if no path can be found
   return None