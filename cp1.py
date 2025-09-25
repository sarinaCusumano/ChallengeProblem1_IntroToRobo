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
from collections import OrderedDict

from Conflict_Resolution import conflict_resolution


# Generic Robot class containing core functionality
class Robot:

    # CHANGED: added 'rid' so each robot gets a stable ID for tie-breaks
    def __init__(self, x, y, g_x, g_y, n, rid):  # CHANGED
        self.x = x
        self.y = y
        self.goal_x = g_x
        self.goal_y = g_y
        self.matrixSize = n
        self.potential = self.generateMatrix()
        self.distance = self.potential[self.y][self.x]
        self.timestepComplete = False
        self.robot_id = rid

    def generateMatrix(self):
        # Initialize the matrix with largest possible value
        max_distance = self.matrixSize * self.matrixSize
        matrix = np.full((self.matrixSize, self.matrixSize), max_distance, dtype=np.int32)
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
                if matrix[curr_y][curr_x - 1] > curr_val + 1:
                    matrix[curr_y][curr_x - 1] = curr_val + 1
                    points.put((curr_x - 1, curr_y))
            # Examine right:
            if curr_x + 1 < self.matrixSize:
                if matrix[curr_y][curr_x + 1] > curr_val + 1:
                    matrix[curr_y][curr_x + 1] = curr_val + 1
                    points.put((curr_x + 1, curr_y))
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

        #CHANGE TO WORK FUNCTIONALITY
        l_x = self.x - 1
        r_x = self.x + 1
        u_y = self.y - 1
        d_y = self.y + 1

        candidateMoves = []
        current_value = self.potential[self.y][self.x]
        if l_x >= 0:
            if self.potential[self.y][l_x] < current_value:
                candidateMoves.append([l_x, self.y])
        if r_x < self.matrixSize:
            if self.potential[self.y][r_x] < current_value:
                candidateMoves.append([r_x, self.y])
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

    # CHANGED: pass rid through to base class
    def __init__(self, x, y, g_x, g_y, n, t, rid):
        super().__init__(x, y, g_x, g_y, n, rid)
        self.type = t
        self.incompatible = {'h', 'd'}

    def getType(self):
        return self.type


class Quadrotor(Robot):
    # CHANGED: pass rid through to base class
    def __init__(self, x, y, g_x, g_y, n, t, rid):
        super().__init__(x, y, g_x, g_y, n, rid)
        self.type = t
        self.incompatible = {'q'}

    def getType(self):
        return self.type


class DifferentialDrive(Robot):
    # CHANGED: pass rid through to base class
    def __init__(self, x, y, g_x, g_y, n, t, rid):
        super().__init__(x, y, g_x, g_y, n, rid)
        self.type = t
        self.incompatible = {'h', 'd'}

    def getType(self):
        return self.type


class Node:

    def __init__(self):
        self.occupancy = []  # CHANGED: now stores Robot objects (not type strings)

    def add_robot(self, robo):
        self.occupancy.append(robo)  # CHANGED: add Robot object

    def remove_robot(self, robo):
        self.occupancy.remove(robo)  # CHANGED: remove Robot object

    def valid_add(self, robo):
        # CHANGED: check incompatibility in both directions using robot.getType()
        for robot in self.occupancy:
            if robot.getType() in robo.incompatible or robo.getType() in robot.incompatible:
                return False
        return True


class Grid:

    #List of robot objects
    #List of dictinary values


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

        # Pre-sample unique start cells so no two robots start in the same spot
        cells = [(x, y) for y in range(n) for x in range(n)]
        starts = random.sample(cells, 2 * n)

        # CHANGED: iterate over unique starts and assign rid as the loop index
        for rid, (random_x, random_y) in enumerate(starts):
            type_options = ['h', 'd', 'q']
            index_options = list(range(n))

            # Grab a random type
            random_type = random.choice(type_options)

            # We now have a valid robot type and a valid location (unique via 'starts')
            # Now let's grab a random goal position
            while True:
                random_g_x = random.choice(index_options)
                random_g_y = random.choice(index_options)
                if random_g_x != random_x or random_g_y != random_y:
                    break

            # We now have a valid type, a valid random position, and a valid goal position
            # Let's create the robot
            if random_type == 'h':
                robot = Humanoid(random_x, random_y, random_g_x, random_g_y, n, random_type, rid)  # CHANGED
            elif random_type == 'd':
                robot = DifferentialDrive(random_x, random_y, random_g_x, random_g_y, n, random_type, rid)  # CHANGED
            else:
                robot = Quadrotor(random_x, random_y, random_g_x, random_g_y, n, random_type, rid)  # CHANGED

            # Add to the max heap priority queue:
            self.robotQueue.put(robot)

            # Place the robot on the grid of nodes
            self.nodes[random_y][random_x].add_robot(robot)  # CHANGED: store Robot object (not type)

    # ============================
    # Intentions --> conflict resolution --> move
    # ============================

    def collect_reservations(self):
        reservation_list = []
        for robot in sorted(self.robotQueue.queue):  # preserve Q order
            robo_dict = {"robot": robot, "reservation_pos": robot.findMoveCandidates()}
            reservation_list.append(robo_dict)

        # two different ordered dictionaries
        primary_dict = OrderedDict()
        secondary_dict = OrderedDict()

        for entry in reservation_list:  # already in priority order
            robot = entry["robot"]
            cands = entry["reservation_pos"]

            if not cands:
                # stay in place if no candidates
                primary_dict[robot] = (robot.get_x(), robot.get_y())
                secondary_dict[robot] = []
            else:
                # first candidate = primary
                first = tuple(cands[0])
                rest = [tuple(c) for c in cands[1:]]
                primary_dict[robot] = first
                secondary_dict[robot] = rest

        return primary_dict, secondary_dict

    def resolve_conflicts(self, reservations_primary, reservations_secondary):
        """
        intentions: dict mapping target_pos (x, y) -> list of Robot objects
          Robot API used: getType() in {'q','d','h'}, get_x(), get_y(), robot_id, potential[y][x]

        Returns:
          dict {winner_id: target_pos}, list [loser_ids]
        """
        winners = {}
        losers = []


        # remaining distance from potential field at current position
        def dist(r):
            return r.potential[r.get_y()][r.get_x()]

        # match-case resolver using 'q','h','d'
        def conflict_resolution(r1, r2):
            t1, t2 = r1.getType(), r2.getType()
            match (t1, t2):
                case ('q', 'q'):
                    return r1.robot_id if dist(r1) > dist(r2) else r2.robot_id
                case ('d', 'd'):
                    return r1.robot_id if dist(r1) > dist(r2) else r2.robot_id
                case ('h', 'h'):
                    return r1.robot_id if dist(r1) > dist(r2) else r2.robot_id
                case ('d', 'h') | ('h', 'd'):
                    return r1.robot_id if dist(r1) > dist(r2) else r2.robot_id
                case ('q', 'h') | ('h', 'q'):
                    return 0
                case ('q', 'd') | ('d', 'q'):
                    return 0

        pos_to_robots_dict = OrderedDict()
        # regroup reservations_primary
        for robot, pos in reservations_primary.items():
            pos_to_robots_dict.setdefault(pos, []).append(robot)

        # ---- primary resolution  ----
        for target_pos, contenders in pos_to_robots_dict.items():  # now contenders is a list of Robots
            n = len(contenders)
            if n == 0:
                continue
            elif n == 1:
                winners[contenders[0].robot_id] = target_pos
            elif n == 2:
                rid = conflict_resolution(contenders[0], contenders[1])
                if rid == 0:  # no conflict → both can go
                    for r in contenders:
                        winners[r.robot_id] = target_pos
                else:
                    winners[rid] = target_pos
                    for r in contenders:
                        if r.robot_id != rid:
                            losers.append(r.robot_id)
            else:
                current = contenders[0]
                for r in contenders[1:]:
                    rid = conflict_resolution(current, r)
                    if rid == 0:
                        winners[current.robot_id] = target_pos
                        winners[r.robot_id] = target_pos
                    elif rid == r.robot_id:
                        losers.append(current.robot_id)
                        current = r
                    else:
                        losers.append(r.robot_id)
                if current.robot_id not in winners:
                    winners[current.robot_id] = target_pos

        # ---- secondary pass: let losers try their alternates if OPEN  ----
        used_positions = set(winners.values())

        # keep only secondary entries for robots that lost; preserve order

        filtered_secondary = OrderedDict(
            (robot, positions)
            for robot, positions in reservations_secondary.items()
            if robot.robot_id in losers
        )

        remaining_losers = []
        for robot, alt_positions in filtered_secondary.items():
            rid = robot.robot_id

            # if this robot already got a primary win skip
            if rid in winners:
                continue

            placed = False
            # normalize any [x,y] to (x,y)
            for pos in (tuple(p) for p in alt_positions):
                if pos not in used_positions:
                    winners[rid] = pos  # assign exactly one new space
                    used_positions.add(pos)
                    placed = True
                    break

            if not placed:
                remaining_losers.append(rid)

        return winners, remaining_losers

    def apply_moves(self, winners, losers):
        """
        winners: dict {robot_id -> (x, y)}
        losers:  list [robot_id, ...]
        """
        robots = list(self.robotQueue.queue)
        by_id = {r.robot_id: r for r in robots}

        completed_ids = set()

        # Apply winning moves
        for rid, (nx, ny) in winners.items():
            r = by_id.get(rid) #get ID
            ox, oy = r.get_x(), r.get_y() #get old position

            # remove from old node if present
            try:
                self.nodes[oy][ox].remove_robot(r) #remove robot from old node
            except ValueError:
                pass

            # If this move reaches the goal, do NOT add back to any node
            if (nx, ny) == (r.get_goal_x(), r.get_goal_y()):
                r.set_x(nx)
                r.set_y(ny)
                completed_ids.add(rid)
                continue

            # otherwise place at new node and update coords
            self.nodes[ny][nx].add_robot(r)
            r.set_x(nx)
            r.set_y(ny)

        #Recompute distances
        for r in robots:
            # skip completed robots: they’re off the board
            if r.robot_id in completed_ids:
                continue
            r.distance = r.potential[r.get_y()][r.get_x()]

        #Rebuild the priority queue with only non-completed robots
        new_q = queue.PriorityQueue()
        for r in robots:
            if r.robot_id not in completed_ids:
                new_q.put(r)
        self.robotQueue = new_q


    def timestep(self, n):
        """
        One global plan-then-move tick:
        - intentions = collect_intentions()
            -if any robots intend to go to a space without any conflicts
            -send the rest to conflict_resolution
        - winners = resolve_conflicts(intentions)
        - apply_moves(winners)
        """
        #Call collect intentions
        reservation_dict_primary, reservation_dict_secondary = self.collect_reservations()
        #Resolve conflict
        winners, losers = self.resolve_conflicts(reservation_dict_primary, reservation_dict_secondary)
        # visualize to show graph with current time step
        myGrid.visualize(n)
        #apply moves (only with final list)
        self.apply_moves(winners, losers)



    # run loop; leave as outline per instructions
    def run_until_done(self):
        """
        Repeat timestep() until all robots reach their goals.
        """

        #while robots on grid - run timestep
        n = 0
        while not self.robotQueue.empty():
            self.timestep(n)
            n += 1

        # visualize to show graph with current time step
        myGrid.visualize(n)


    def visualize(self, n):

        type_color = {'q': 'tab:red', 'd': 'tab:green', 'h': 'tab:blue'}
        type_marker = {'q': 'X', 'd': '^', 'h': 'o'}
        type_label = {'q': 'Quadrotor', 'd': 'DifferentialDrive', 'h': 'Humanoid'}

        # Draw all robots: start, goal, dashed connector
        for robot in list(self.robotQueue.queue):
            t = robot.getType()
            c = type_color[t]
            m = type_marker[t]
            l = type_label[t]
            x_coords = [robot.get_x(), robot.get_goal_x()]
            y_coords = [robot.get_y(), robot.get_goal_y()]

            plt.plot(x_coords, y_coords, '--', color=c)
            plt.plot(x_coords[0], y_coords[0], m, color=c, markersize=12, label=l)
            plt.plot(x_coords[1], y_coords[1], m, color='gold')


        plt.title("Robot Grid Navigation - timestep " + str(n)) # Added Title - updated with timestep number
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.grid(True)
        plt.xticks(range(self.n))
        plt.yticks(range(self.n))
        plt.axis('equal')
        plt.tight_layout(rect=[0.0, 0.0, 1.0, 1.0])  # leave extra space on the right for the legend
        #plt.savefig("warehouse.png", dpi=200, bbox_inches="tight")
        plt.xlim(-1, self.n)
        plt.ylim(-1, self.n)
        plt.show(block=False)
        plt.pause(3)
        plt.close()



myGrid = Grid(5)
myGrid.run_until_done()