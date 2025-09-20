#################################################################################################################################
### ROBO 5000 : Introduction to Robotics
### Challenge Problem 1: Robot Warehouse Management
### Authors: 
#################################################################################################################################

# Import required packages
import numpy as np
import queue
import random
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.legend_handler import HandlerTuple


# Generic Robot class containing core functionality
class Robot:

    def __init__(self, x, y, g_x, g_y, n):
        self.x = x
        self.y = y
        self.goal_x = g_x
        self.goal_y = g_y
        self.matrixSize = n
        self.potential = self.generateMatrix()
        self.distance = self.potential[self.y][self.x]
        self.timestepComplete = False
    
    def generateMatrix(self):
        # Initialize the matrix with largest possible value
        max_distance = self.matrixSize * self.matrixSize
        matrix = np.full((self.matrixSize,self.matrixSize), max_distance, dtype=np.int32)
        points = queue.Queue()
        matrix[self.goal_y][self.goal_x] = 0
        points.put((self.goal_x, self.goal_y))
        # Effectively run BFS on the matrix, calculating each node's distance from the goal
        while not points.empty():
            # Dequeue the first element and explore valid neighbors
            curr = points.get()
            curr_x = curr[0]
            curr_y = curr[1]
            curr_val = matrix[curr_y][curr_x]
            # Examine left:
            if curr_x - 1 >= 0:
                if matrix[curr_y][curr_x-1] > curr_val + 1:
                    matrix[curr_y][curr_x-1] = curr_val + 1
                    points.put((curr_x-1,curr_y))
            # Examine right:
            if curr_x + 1 < self.matrixSize:
                if matrix[curr_y][curr_x + 1] > curr_val + 1:
                    matrix[curr_y][curr_x + 1] = curr_val + 1
                    points.put((curr_x+1, curr_y))
            # Examine up:
            if curr_y - 1 >= 0:
                if matrix[curr_y - 1][curr_x] > curr_val + 1:
                    matrix[curr_y - 1][curr_x] = curr_val + 1
                    points.put((curr_x, curr_y - 1))
            # Examine down:
            if curr_y + 1 < self.matrixSize:
                if matrix[curr_y + 1][curr_x] > curr_val + 1:
                    matrix[curr_y + 1][curr_x] = curr_val + 1
                    points.put((curr_x, curr_y + 1))
        return matrix
    
    # Helper function to validate the potential matrix
    def printMatrix(self):
        for i in range(self.matrixSize):
            for j in range(self.matrixSize):
                print(f"{self.potential[i][j]}", end=' ')

    def findMoveCandidates(self):
        # The goal of this function is to determine the coordinates of neighbors lying on a shortest path
        # Implementation is pretty simple, we just need to check left, right, up, down
        l_x = self.x - 1
        r_x = self.x + 1
        u_y = self.y - 1
        d_y = self.y + 1

        candidateMoves = []
        current_value = self.potential[self.y][self.x]
        if l_x >= 0:
            if self.potential[self.y][l_x] < current_value:
                candidateMoves.append([l_x,self.y])
        if r_x < self.matrixSize:
            if self.potential[self.y][r_x] < current_value:
                candidateMoves.append([r_x,self.y])
        if u_y >= 0:
            if self.potential[u_y][self.x] < current_value:
                candidateMoves.append([self.x, u_y])
        if d_y < self.matrixSize:
            if self.potential[d_y][self.x] < current_value:
                candidateMoves.append([self.x, d_y])
        return candidateMoves
      
    # __lt__ Method for comparing instances of the class
    def __lt__(self, other):
        return self.distance > other.distance
    
    # Getters and Setters
    def getDistance(self):
        return self.distance
    
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y
    
    def get_goal_x(self):
        return self.goal_x

    def get_goal_y(self):
        return self.goal_y

    def get_timestepComplete(self):
        return self.timestepComplete
    
    def set_timestepComplete(self, input):
        self.timestepComplete = input

    def set_x(self, x):
        self.x = x
    
    def set_y(self, y):
        self.y = y


class Humanoid(Robot):

    def __init__(self, x, y, g_x, g_y, n, t):
        super().__init__(x, y, g_x, g_y, n)
        self.type = t
        self.incompatible = {'h', 'd'}

    def getType(self):
        return self.type

class Quadrotor(Robot):
    def __init__(self, x, y, g_x, g_y, n, t):
        super().__init__(x, y, g_x, g_y, n)
        self.type = t
        self.incompatible = {'q'}

    def getType(self):
        return self.type

class DifferentialDrive(Robot):
    def __init__(self, x, y, g_x, g_y, n, t):
        super().__init__(x, y, g_x, g_y, n)
        self.type = t
        self.incompatible = {'h', 'd'}

    def getType(self):
        return self.type

class Node:
    
    def __init__(self):
        self.occupancy = []
    
    def add_robot(self, robo):
        self.occupancy.append(robo)
    
    def remove_robot(self, robo):
        self.occupancy.remove(robo)

    def valid_add(self, robo):
        for robot in self.occupancy:
            if robot.getType() in robo.incompatible:
                return False
        return True

    def valid_add_type_only(self, robo_type):
        for robot in self.occupancy:
            if robo_type in robot.incompatible:
                return False
        return True

class Grid:

    def __init__(self, n):
        # In order to set up our grid, we need to do the following:
        #       Create an nxn matrix of nodes
        #       Place 2n random robots on the grid
        #           For each robot do the following:
        #               Generate a random type
        #               Generate a random position
        #               Ensure position and type are compatible
        #               Generate a random goal position
        #               Ensure goal position is different from current position
        #               Call the constructor for the given robot and append to a max heap priority queue

        self.n = n
        # Let's start by creating the nxn matrix of nodes
        self.nodes = []
        for i in range(n):
            row = []
            for j in range(n):
                row.append(Node())
            self.nodes.append(row)
        
        # Let's create our max heap priority queue that will store the robots
        self.robotQueue = queue.PriorityQueue()
        for i in range(2 * n):
            type_options = ['h', 'd', 'q']
            index_options = list(range(n))

            # Grab a random type 
            random_type = random.choice(type_options)

            # Grab a random position and validate
            while True:
                random_x = random.choice(index_options)
                random_y = random.choice(index_options)
                if self.nodes[random_y][random_x].valid_add(random_type):
                    break
            
            # We now have a valid robot type and a valid location
            # Now let's grab a random goal position
            while True:
                random_g_x = random.choice(index_options)
                random_g_y = random.choice(index_options)
                if random_g_x != random_x or random_g_y != random_y:
                    break
            
            # We now have a valid type, a valid random position, and a valid goal position
            # Let's create the robot
            if random_type == 'h':
                robot = Humanoid(random_x, random_y, random_g_x, random_g_y, n, random_type)
            elif random_type == 'd':
                robot = DifferentialDrive(random_x, random_y, random_g_x, random_g_y, n, random_type)
            else:
                robot = Quadrotor(random_x, random_y, random_g_x, random_g_y, n, random_type)
            
            # Add to the max heap priority queue:
            self.robotQueue.put(robot)

            # Place the robot on the grid of nodes
            self.nodes[random_y][random_x].add_robot(random_type)
    
    def timestep(self):
        # Strategy for implementing timestep resolution
        # We initially have all robots in a max-heap priority queue
        # We will remove the robot with the highest priority based on distance
        # Examine the node's along that robot's shortest path
        # If a node is available, simply move the robot to that node
        # If no nodes are available along the shortest path, send shortest path nodes to a function to check for swapping
        # Mark the robot as completed for this timestep and append to auxiliary array
        # Note that in the event of a swap, we simply mark the robot as complete for the timestep and skip everything when dequeued
        # Once the priority queue is empty, the timestep is complete
        # Iterate through and set all timestep completion flags to false and enqueue
        # In the event of a swap or a clean move, prioritize a clean move
        # In the event of two open options, implement a density function and move to the cell with the lowest neighboring density

        # Create the auxiliary array that will store completed robots
        aux = []
        # Loop while the priority queue is not empty
        while not self.robotQueue.empty():
            # Grab the highest priority robot from the front of the queue
            current = self.robotQueue.get()

            # If the robot is already marked as complete for this timestep (was part of earlier swap) append and continue
            if current.timestepComplete:
                aux.append(current)
                continue
                
            # If the robot is not complete for this timestep we need to move it
            # Examine the robot's 2-4 neighbors and identify candidates for moving (cells with less distance)
            candidates = current.findMoveCandidates()

            # We now have all possible shortest path moves for our current robot
            # The flow from this point on:
            # If we only have one candidate and the cell is valid for our current type: move the robot
            # If we only have one candidate and the cell is not valid for our current type: analyze for a swap
            # If we have two candidates and both are valid for our current type: calculate the minimum neighboring density between the two
            # If we have two candidates and only one is valid for our current type: move to the valid node (prioritize movement over swaps)
            # If we have two candidates and neither are valid: check both for swaps
            if len(candidates) == 1:
                # Check if the cell is valid for our robot
                if self.nodes[candidates[0][1]][candidates[0][0]].valid_add(current):
                    # The cell is valid. Let's move the robot into the cell, mark as complete, and continue
                    self.nodes[candidates[0][1]][candidates[0][0]].add_robot(current.getType())
                    self.nodes[current.get_y()][current.get_x()].remove_robot(current.getType())
                    current.set_timestepComplete(True)
                    current.set_x(candidates[0][0])
                    current.set_y(candidates[0][1])
                    if not (current.get_x() == current.get_goal_x() and current.get_y() == current.get_goal_y()):
                        aux.append(current)
                # If the cell is not valid, we need to implement a function to check if there is a swap available
                # Need to make an edit to the code: node needs to store the currently occupying robot, not just its type
            elif len(candidates) == 2:
                # Handle the case where both cells are valid
                # Handle the case where one is valid and one is not
                # Handle the case where both are invalid

                if self.nodes[candidates[0][1]][candidates[0][0]].valid_add(current):
                    # Handle the case where both are valid
                    if self.nodes[candidates[1][1]][candidates[1][0]].valid_add(current):
                        # Need to send to a density calculator function here
                    # Handle the case where only one of the two is valid
                    else:
                        # We have a valid movement option, so let's take it
                else:
                    # Handle the case where only one is valid
                    if self.nodes[candidates[1][1]][candidates[1][0]].valid_add(current):

                    # Handle the case where both are invalid
                    else:
                        



    
    def visualize(self):

        type_color = {'q': 'tab:red', 'd': 'tab:green', 'h': 'tab:blue'}
        type_marker = {'q': 'X', 'd': '^', 'h': 'o'}
        type_label = {'q': 'Quadrotor', 'd': 'DifferentialDrive', 'h': 'Humanoid'}

        # Draw all robots: start, goal, dashed connector
        for robot in list(self.robotQueue.queue): 
            t  = robot.getType()
            c  = type_color[t]
            m = type_marker[t]
            l = type_label[t]
            x_coords = [robot.get_x(),robot.get_goal_x()]
            y_coords = [robot.get_y(),robot.get_goal_y()]

            plt.plot(x_coords,y_coords, '--', color=c)
            plt.plot(x_coords[0], y_coords[0],m, color=c,markersize=12, label=l)
            plt.plot(x_coords[1],y_coords[1],m, color='gold') 

        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.grid(True)
        plt.xticks(range(-1, self.n))
        plt.yticks(range(-1, self.n))
        plt.axis('equal')
        plt.savefig("warehouse.png", dpi=200, bbox_inches="tight")


myGrid = Grid(5)
myGrid.visualize()
            


        

            

            
            



