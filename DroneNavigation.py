# Challenge Problem 1

import numpy as np
import random as r
from collections import deque
import math

class Robot:
    def __init__(self, startX, startY, goalX, goalY, n_matrix):
        self.startX = startX
        self.startY = startY
        self.goalX = goalX
        self.goalY = goalY
        self.n_matrix = n_matrix

        self.matrix = np.zeros([self.n_matrix, self.n_matrix])
        self.currentX = self.startX
        self.currentY = self.startY
    
    def get_startX(self):
        return self.xPos

    def set_currentX(self, x):
        self.currentX = x

    def get_startY(self):
        return self.yPos

    def set_currentY(self, y):
        self.currentY = y

    def get_current_position(self):
        return [self.currentY, self.currentX]

    def get_goalX(self):
        return self.goalX

    def set_goalX(self, x):
        self.goalX = x

    def get_goalY(self):
        return self.goalY

    def set_goalY(self, y):
        self.goalY = y

    def get_matrix(self):
        return self.matrix

    def update_matrix(self):
        self.matrix[self.currentX, self.currentY] = 1

    def __str__(self):
        return("This bot's position is [" + str(startX) + "," + str(startY) + "]")

    def get_distance(self):
        self.dist = math.sqrt((self.startX-self.goalX)**2+(self.startY-self.goalY)**2)
        return self.dist
        


class Humanoid(Robot):
    def __init__(self, startX, startY, goalX, goalY, n_matrix):
        super().__init__(startX, startY, goalX, goalY, n_matrix)
        self.startX = startX
        self.startY = startY
        self.goalX = goalX
        self.goalY = goalY
        self.n_matrix = n_matrix

    def __str__(self):
        return ("Humanoid that starts at " + str(self.startX) + "," + str(self.startY) + " and goes to " + str(self.goalX) + "," + str(self.goalY))

class DifferentialDrive(Robot):
    def __init__(self, startX, startY, goalX, goalY, n_matrix):
        super().__init__(startX, startY, goalX, goalY, n_matrix)
        self.startX = startX
        self.startY = startY
        self.goalX = goalX
        self.goalY = goalY

    def __str__(self):
        return ("Differential Drive that starts at " + str(self.startX) + "," + str(self.startY) + " and goes to " + str(self.goalX) + "," + str(self.goalY))

class Quadrotor(Robot):
    def __init__(self, startX, startY, goalX, goalY, n_matrix):
        super().__init__(startX, startY, goalX, goalY, n_matrix)
        self.startX = startX
        self.startY = startY
        self.goalX = goalX
        self.goalY = goalY

    def __str__(self):
        return ("Quadrotor that starts at " + str(self.startX) + "," + str(self.startY) + " and goes to " + str(self.goalX) + "," + str(self.goalY))

class Grid:
    def __init__(self, size):
        self.size=size

        self.grid = np.empty((size,size), Node)
        for r in range(size):
            for c in range(size):
                self.grid[r,c] = Node(True)
        

# Creates a Node Class so that the attriutes of each node in the grid can be accessed
class Node:
    def __init__(self, state: True):
        self.state = state # Open = True, Occupied = False
        #self.bot = bot # Bot in that node 
        
##    def getBot(self):
##        return self.bot
##
##    def setBot(self, b):
##        self.bot = b

    def getState(self):
        return self.state

    def setState(self, s):
        self.state = s


if __name__ == "__main__":
    
    num = int(input("Please enter a number (between 3-10): "))

    grid = Grid(num)

    # 2d list with the type of bot, and then it's distance to goal
    bots = []
    
    # Creates bots based on input value, randomly creates start and goal locations, adds them to Grid
    nums = list(range(0,num*2))
    r.shuffle(nums)
    print(nums)
    nums1 = list(range(0,num*2))
    r.shuffle(nums1)
    nums2 = list(range(0,num*2))
    r.shuffle(nums2)
    nums3 = list(range(0,num*2))
    r.shuffle(nums3)
    for i in range(0,num*2):
        bot = r.randint(1,3)
        startX = nums[i]
        startY = nums1[i]
        goalX = nums2[i]
        goalY = nums3[i]
        if bot == 1:
            h = Humanoid(startX, startY, goalX, goalY, num)
            bots.append([h, h.get_distance()])
        elif bot ==2:
            d = DifferentialDrive(startX, startY, goalX, goalY, num)
            bots.append([d, d.get_distance()])
        else:
            q = Quadrotor(startX, startY, goalX, goalY, num)
            bots.append([q, q.get_distance()])

    sorted_bots = sorted(bots, key=lambda x: x[1], reverse=True)

    #test sorted list
    print(sorted_bots[0][1])
    print(sorted_bots[1][1])
    print(sorted_bots[2][1])

    
    


        
        
    


    
    
