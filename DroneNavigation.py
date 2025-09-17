# Challenge Problem 1

import numpy as np
import random as r

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
        


class Humanoid(Robot):
    def __init__(self, startX, startY, goalX, goalY, n_matrix):
        super().__init__(startX, startY, goalX, goalY, n_matrix)

class DifferentialDrive(Robot):
    def __init__(self, startX, startY, goalX, goalY, n_matrix):
        super().__init__(startX, startY, goalX, goalY, n_matrix)

class Quadrotor(Robot):
    def __init__(self, startX, startY, goalX, goalY, n_matrix):
        super().__init__(startX, startY, goalX, goalY, n_matrix)

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

    bots = []
    
    # Creates bots based on input value, randomly creates start and goal locations, adds them to Grid
    for i in range(0,num*2):
        bot = r.randint(1,3)
        startX = r.randint(0,num)
        startY = r.randint(0,num)
        goalX = r.randint(0,num)
        goalY = r.randint(0,num)
        if bot == 1:
            h = Humanoid(startX, startY, goalX, goalY, num)
            bots.append(h)
        elif bot ==2:
            d = DifferentialDrive(startX, startY, goalX, goalY, num)
            bots.append(d)
        else:
            q = Quadrotor(startX, startY, goalX, goalY, num)
            bots.append(q)

    for i in range(0, num*2):
        print(str(bots[i]))


        
        
    


    
    
