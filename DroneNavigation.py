# Challenge Problem 1

Class Robot:
    def __init__(self, xPos, yPos, goalX, goalY, matrix, distance)
        xPos = self.xPos
        yPos = self.yPos
        goalX = self.goalX
        goalY = self.goalY
        matrix = self.matrix
        distance = self.distance

    def get_xPos(self):
        return self.xPos

    def set_xPos(self, x):
        self.xPos = x

    def get_yPos(self):
        return self.yPos

    def set_yPos(self, y):
        self.yPos = y

    def get_goalX(self):
        return self.goalX

    def set_goalX(self, x):
        self.goalX = x

    def get_goalY(self):
        return self.goalY

    def set_goalY(self, y):
        self.goalY = y

    


Class Humanoid(Robot):
    def __init__(self)

Class Quadrotor(Robot):
    def __init__(self)

Class Drone(Robot):
    def __init__(self):

Class Grid:
    def __init__(self):

Class Node:
    def __init__(self, state, occupied_bot, reserved_bot):
        self.state = state # Open, occupied, reserved
        self.occupied_bot = occupied_bot # The bot that is occupied at that node 
        self.reserved_bot = reserved_bot # The bot that has the node reserved
        
    def getBot(self):
        return self.bot

    def setBot(self, b):
        self.bot = b

    def getState(self):
        return self.state

    def setState(self, s):
        self.state = s

    

    


    
    
