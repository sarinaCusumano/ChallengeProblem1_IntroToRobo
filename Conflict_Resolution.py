#conflict resolution
#Drones can occupy the same cell as humanoid or differential drive robots.
#Humanoid robots cannot occupy the same cell as other humanoids.
# Differential drive robots cannot share the same cell as other differential drive robots.
# Differential drive robots cannot share the same cell as humanoids.
#Drones cannot occupy the same cell as other drones.
from winreg import QueryReflectionKey

drone = 1
differential = 8
humanoid = 5


combo1 = 2 #drone and drone - conflict
combo2 = 9 #drone and differential - no conflict
combo3 = 6 #drone and humanoid - no conflict
combo4 = 16 #differential and differential - conflict
combo5 = 13 #differential and humanoid - conflict
combo6 = 10 #humanoid and humanoid - conflict


def conflict_resolution(robot1, robot2):
    conflictcombo = robot1[0] + robot2[0]
    no_conflict = 0
    match conflictcombo:
        case 2:
            if robot1[3] > robot2[3]:
                return robot1[4]
            else:
                return robot2[4]
        case 9:
            return no_conflict
        case 6:
            return no_conflict
        case 16:
            if robot1[3] > robot2[3]:
                return robot1[4]
            else:
                return robot2[4]
        case 13:
            if robot1[3] > robot2[3]:
                return robot1[4]
            else:
                return robot2[4]
        case 10:
            if robot1[3] > robot2[3]:
                return robot1[4]
            else:
                return robot2[4]




robot0 = [humanoid, (0,1), (0,2), 2, 1] #type, current position, goal position, distance needed, identifier
robot1 = [drone, (1,2), (0,2), 4, 2] #type, current position, goal position, distance needed, identifier
robot2 = [differential, (1,2), (0,2), 4, 3] #type, current position, goal position, distance needed, identifier
robot3 = [humanoid, (0,1), (0,2), 2, 4] #type, current position, goal position, distance needed, identifier
robot4 = [drone, (1,2), (0,2), 4, 2] #type, current position, goal position, distance needed, identifier
robot5 = [differential, (1,2), (0,2), 4, 3] #type, current position, goal position, distance needed, identifier
robot6 = [humanoid, (0,1), (0,2), 2, 1] #type, current position, goal position, distance needed, identifier
robot7 = [drone, (1,2), (0,2), 4, 2] #type, current position, goal position, distance needed, identifier
robot8 = [differential, (1,2), (0,2), 4, 3] #type, current position, goal position, distance needed, identifier
robot9 = [humanoid, (1,2), (0,2), 4, 3] #type, current position, goal position, distance needed, identifier

robots = [robot0, robot1, robot2, robot3, robot4, robot5, robot6, robot7,robot8, robot9]
current_positions = [robot0[1], robot1[1], robot2[1], robot3[1], robot4[1], robot5[1], robot6[1], robot7[1], robot8[1], robot9[1]]
desired_positions = [robot0[2], robot1[2], robot2[2], robot3[2], robot4[2], robot5[2], robot6[2], robot7[2], robot8[2], robot9[2]]



print(conflict_resolution(robot3, robot6))





