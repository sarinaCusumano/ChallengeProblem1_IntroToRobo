#################################################################################################################################
### ROBO 5000 : Introduction to Robotics
### Challenge Problem 1: Robot Warehouse Management
### Authors: Marc Friedman, Joseph Hanley, Sarina Cusumano
#################################################################################################################################

import numpy as np
import queue
import random
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from collections import OrderedDict




# -------------------------------------------------------------------------------------------------
# Robot
# Core agent with grid coordinates, goal, and a BFS-derived potential field (distance-to-goal).
# finds shortest-path candidate moves and supports priority ordering.
class Robot:


    def __init__(self, x, y, g_x, g_y, n, rid):
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
        # Build a BFS-based distance field (potential) from goal to every cell.
        max_distance = self.matrixSize * self.matrixSize
        matrix = np.full((self.matrixSize, self.matrixSize), max_distance, dtype=np.int32)
        points = queue.Queue()
        matrix[self.goal_y][self.goal_x] = 0
        points.put((self.goal_x, self.goal_y))
        # Effectively run BFS on the matrix, calculating each node's distance from the goal.

        while not points.empty():
            # Dequeue the next cell and relax its four neighbors.
            curr = points.get()
            curr_x = curr[0]
            curr_y = curr[1]
            curr_val = matrix[curr_y][curr_x]
            # Examine left.
            if curr_x - 1 >= 0:
                if matrix[curr_y][curr_x - 1] > curr_val + 1:
                    matrix[curr_y][curr_x - 1] = curr_val + 1
                    points.put((curr_x - 1, curr_y))
            # Examine right.
            if curr_x + 1 < self.matrixSize:
                if matrix[curr_y][curr_x + 1] > curr_val + 1:
                    matrix[curr_y][curr_x + 1] = curr_val + 1
                    points.put((curr_x + 1, curr_y))
            # Examine up.
            if curr_y - 1 >= 0:
                if matrix[curr_y - 1][curr_x] > curr_val + 1:
                    matrix[curr_y - 1][curr_x] = curr_val + 1
                    points.put((curr_x, curr_y - 1))
            # Examine down.
            if curr_y + 1 < self.matrixSize:
                if matrix[curr_y + 1][curr_x] > curr_val + 1:
                    matrix[curr_y + 1][curr_x] = curr_val + 1
                    points.put((curr_x, curr_y + 1))
        return matrix

    # Helper to print the potential matrix for debugging.
    def printMatrix(self):
        for i in range(self.matrixSize):
            for j in range(self.matrixSize):
                print(f"{self.potential[i][j]}", end=' ')

    # Determine the 4-neighborhood cells that advance along the shortest path.
    def findMoveCandidates(self):

        l_x = self.x - 1
        r_x = self.x + 1
        u_y = self.y - 1
        d_y = self.y + 1

        candidateMoves = []
        current_value = self.potential[self.y][self.x]
        if l_x >= 0:
            if self.potential[self.y][l_x] < current_value:
                candidateMoves.append([l_x, self.y])
        if u_y >= 0:
            if self.potential[u_y][self.x] < current_value:
                candidateMoves.append([self.x, u_y])
        if r_x < self.matrixSize:
            if self.potential[self.y][r_x] < current_value:
                candidateMoves.append([r_x, self.y])
        if d_y < self.matrixSize:
            if self.potential[d_y][self.x] < current_value:
                candidateMoves.append([self.x, d_y])
        return candidateMoves

    # Invert comparison so PriorityQueue behaves as a max-heap on distance.
    def __lt__(self, other):
        return self.distance > other.distance

    # getters and setters.
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


# -------------------------------------------------------------------------------------------------
# Humanoid
# Ground robot subtype.
class Humanoid(Robot):

    def __init__(self, x, y, g_x, g_y, n, t, rid):
        super().__init__(x, y, g_x, g_y, n, rid)
        self.type = t
        self.incompatible = {'h', 'd'}

    def getType(self):
        return self.type


# -------------------------------------------------------------------------------------------------
# Quadrotor
# Aerial robot subtype.
class Quadrotor(Robot):


    def __init__(self, x, y, g_x, g_y, n, t, rid):
        super().__init__(x, y, g_x, g_y, n, rid)
        self.type = t
        self.incompatible = {'q'}

    def getType(self):
        return self.type


# -------------------------------------------------------------------------------------------------
# DifferentialDrive
# Ground robot subtype.
class DifferentialDrive(Robot):

    def __init__(self, x, y, g_x, g_y, n, t, rid):
        super().__init__(x, y, g_x, g_y, n, rid)
        self.type = t
        self.incompatible = {'h', 'd'}

    def getType(self):
        return self.type


# -------------------------------------------------------------------------------------------------
# Node
# Single grid cell that tracks which Robot objects occupy it.
# Provides add/remove operations
class Node:
    def __init__(self):
        self.occupancy = []

    def can_add(self, r):
        t = r.getType()
        has_q = any(x.getType() == 'q' for x in self.occupancy)
        ground = sum(x.getType() in {'h','d'} for x in self.occupancy)
        return (t == 'q' and not has_q) or (t in {'h','d'} and ground == 0)

    def add_robot(self, r):
        if self.can_add(r):
            self.occupancy.append(r)
            return True
        return False


    def remove_robot(self, robo):
        self.occupancy.remove(robo)

# -------------------------------------------------------------------------------------------------
# Grid
# builds the node lattice, spawns robots with unique starts/goals,
# orchestrates plan→resolve→move timesteps, and maintains the robot priority queue.
# provides visualization and the two-phase (primary/secondary) conflict-resolution pipeline.
class Grid:


    def __init__(self, n):
        self.n = n
        # Create an n×n grid of nodes.
        self.nodes = []
        for i in range(n):
            row = []
            for j in range(n):
                row.append(Node())
            self.nodes.append(row)

        # Use a distance-based priority queue for robots (max-heap via __lt__).
        self.robotQueue = queue.PriorityQueue()

        # Pre-sample unique start cells so no two robots start in the same spot.
        cells = [(x, y) for y in range(n) for x in range(n)]
        starts = random.sample(cells, 2 * n)

        # Create robots with unique IDs, random types, and distinct goals.
        for rid, (random_x, random_y) in enumerate(starts):
            type_options = ['h', 'd', 'q']
            index_options = list(range(n))

            # Choose a random type.
            random_type = random.choice(type_options)
            # Choose a random goal that differs from the start.
            while True:
                random_g_x = random.choice(index_options)
                random_g_y = random.choice(index_options)
                if random_g_x != random_x or random_g_y != random_y:
                    break
            # Instantiate the appropriate robot subclass.
            if random_type == 'h':
                robot = Humanoid(random_x, random_y, random_g_x, random_g_y, n, random_type, rid)  # CHANGED
            elif random_type == 'd':
                robot = DifferentialDrive(random_x, random_y, random_g_x, random_g_y, n, random_type, rid)  # CHANGED
            else:
                robot = Quadrotor(random_x, random_y, random_g_x, random_g_y, n, random_type, rid)  # CHANGED

            # Enqueue the robot by priority.
            self.robotQueue.put(robot)

            # Place the robot on the grid.
            self.nodes[random_y][random_x].add_robot(robot)

    # Determine positions each robot wants to move to along the shortest path.
    def collect_reservations(self):
        reservation_list = []
        for robot in sorted(self.robotQueue.queue):  # Preserve queue order.
            robo_dict = {"robot": robot, "reservation_pos": robot.findMoveCandidates()}
            reservation_list.append(robo_dict)

        # Primary choices to send to conflict resolution (initial winners/losers).
        primary_dict = OrderedDict()
        # Secondary choices retained for losers to try later.
        secondary_dict = OrderedDict()

        # Split each robot’s candidates into primary (first) and secondary (remaining).
        for entry in reservation_list:
            robot = entry["robot"]
            res_pos = entry["reservation_pos"]

            # If no candidates exist, the robot stays put and has no alternates.
            if not res_pos:
                primary_dict[robot] = (robot.get_x(), robot.get_y())
                secondary_dict[robot] = []
            # Otherwise, the first candidate is primary and the rest are alternates.
            else:
                first = tuple(res_pos[0])
                rest = [tuple(c) for c in res_pos[1:]]
                primary_dict[robot] = first
                secondary_dict[robot] = rest

        return primary_dict, secondary_dict

    # takes the primary and secondary reservations and determines list of winners (robots moving) and losers (robots staying)
    def resolve_conflicts(self, reservations_primary, reservations_secondary):

        winners = {}
        losers = []

        # Remaining distance from potential field at current position.
        def dist(r):
            return r.potential[r.get_y()][r.get_x()]

        # Conflict resolver using 'q', 'h', and 'd' type rules.
        # random tie-breaks when distances are equal.
        def conflict_resolution(r1, r2):
            d1 = dist(r1); d2 = dist(r2)
            t1, t2 = r1.getType(), r2.getType()
            match (t1, t2):
                case ('q', 'q'):
                    if d1 > d2:
                        return r1.robot_id
                    if d2 > d1:
                        return r2.robot_id
                    return random.choice([r1.robot_id, r2.robot_id])
                case ('d', 'd'):
                    if d1 > d2:
                        return r1.robot_id
                    if d2 > d1:
                        return r2.robot_id
                    return random.choice([r1.robot_id, r2.robot_id])
                case ('h', 'h'):
                    if d1 > d2:
                        return r1.robot_id
                    if d2 > d1:
                        return r2.robot_id
                    return random.choice([r1.robot_id, r2.robot_id])
                case ('d', 'h') | ('h', 'd'):
                    if d1 > d2:
                        return r1.robot_id
                    if d2 > d1:
                        return r2.robot_id
                    return random.choice([r1.robot_id, r2.robot_id])
                case ('q', 'h') | ('h', 'q'):
                    return 0
                case ('q', 'd') | ('d', 'q'):
                    return 0

        pos_to_robots_dict = OrderedDict()
        # Group primary reservations by target position (pos → list of robots).
        for robot, pos in reservations_primary.items():
            pos_to_robots_dict.setdefault(pos, []).append(robot)

            # swap prioritization (allow 2-cycles; force followers to wait)
            current_positions = {r.robot_id: (r.get_x(), r.get_y()) for r in reservations_primary.keys()}
            swapped_ids = set()
            used_positions = set()

            for robot_a in reservations_primary.keys():
                id_a = robot_a.robot_id
                if id_a in swapped_ids:
                    continue
                tgt_a = reservations_primary[robot_a]

                # find the robot currently at my target
                robot_b = next((rb for rb in reservations_primary.keys()
                                if current_positions[rb.robot_id] == tgt_a), None)
                if robot_b is None:
                    continue

                id_b = robot_b.robot_id
                if id_b in swapped_ids:
                    continue
                tgt_b = reservations_primary[robot_b]

                # mutual desire to each other's cells -> swap
                if tgt_b == current_positions[id_a]:
                    winners[id_a] = tgt_a
                    winners[id_b] = tgt_b
                    swapped_ids.update([id_a, id_b])
                    used_positions.update([tgt_a, tgt_b])

                    # any other contenders for either swap cell must wait this tick
                    for pos in (tgt_a, tgt_b):
                        for contender in pos_to_robots_dict.get(pos, []):
                            cid = contender.robot_id
                            if cid not in swapped_ids and cid not in winners:
                                losers.append(cid)


        # Primary resolution: decide winners for each contested target position.
        for target_pos, contenders in pos_to_robots_dict.items():  # contenders = list of robots wanting the same cell
            n = len(contenders)

            # No contenders: nothing to resolve.
            if n == 0:
                continue

            # One contender: wins automatically.
            elif n == 1:
                winners[contenders[0].robot_id] = target_pos

            # Two contenders: resolve via conflict rules (or allow both if no conflict).
            elif n == 2:
                rid = conflict_resolution(contenders[0], contenders[1])
                if rid == 0:  # No conflict: both succeed.
                    for r in contenders:
                        winners[r.robot_id] = target_pos
                else:  # One winner and one loser.
                    winners[rid] = target_pos
                    for r in contenders:
                        if r.robot_id != rid:
                            losers.append(r.robot_id)

            # Three or more contenders: pick top priority, then (optionally) one compatible partner.
            else:
                # helper: remaining distance (higher = higher priority)
                def dist(r):
                    return r.potential[r.get_y()][r.get_x()]

                # sort by distance, break exact ties randomly
                def key_with_random_tiebreak(r):
                    return (dist(r), random.random())
                ranked = sorted(contenders, key=key_with_random_tiebreak, reverse=True)

                # top winner
                winner = ranked[0]
                winners[winner.robot_id] = target_pos

                # allow at most one compatible sharer:
                # - if winner is ground ('h'/'d'), allow a 'q'
                # - if winner is air ('q'), allow one ground ('h' or 'd')
                wtype = winner.getType()
                need_type = {'q'} if wtype in {'h', 'd'} else {'h', 'd'}

                partner = None
                for r in ranked[1:]:
                    if r.getType() in need_type:
                        partner = r
                        break  # found a compatible sharer; stop searching

                if partner is not None:
                    winners[partner.robot_id] = target_pos

                # everyone else loses
                for r in ranked[1:]:
                    if r is not partner:
                        losers.append(r.robot_id)

        # Secondary resolution: losing robots try alternates if spaces remain open.
        used_positions = set(winners.values())

        # Keep secondary options only for robots that lost; preserve insertion order.
        filtered_secondary = OrderedDict(
            (robot, positions)
            for robot, positions in reservations_secondary.items()
            if robot.robot_id in losers
        )

        remaining_losers = []
        # Each loser tries alternates in order until finding an unused position.
        for robot, alt_positions in filtered_secondary.items():
            rid = robot.robot_id

            # Skip if already assigned during primary resolution.
            if rid in winners:
                continue

            placed = False
            # Normalize alternates to tuples and test availability.
            for pos in (tuple(p) for p in alt_positions):
                # verify the actual cell can accept this robot
                nx, ny = pos
                if pos not in used_positions and self.nodes[ny][nx].can_add(robot):
                    winners[rid] = pos  # Assign one available alternate.
                    used_positions.add(pos)
                    placed = True
                    break

            # If no alternates worked, remain a loser.
            if not placed:
                remaining_losers.append(rid)

        # Return final winners and any robots still unresolved.
        return winners, remaining_losers

    # takes winners list and applies moves
    def apply_moves(self, winners):

        robots = list(self.robotQueue.queue)
        by_id = {r.robot_id: r for r in robots}

        completed_ids = set()

        # Apply winning moves to the grid and update positions.
        # Staged move: first detach all winners from old cells; then place into new cells.
        move_plans = []  # (r, ox, oy, nx, ny)

        for rid, (nx, ny) in winners.items():
            r = by_id.get(rid)  # Get the robot by ID.
            ox, oy = r.get_x(), r.get_y()  # Current position.

            # Remove from the old node if present.
            try:
                self.nodes[oy][ox].remove_robot(r)  # Detach from old cell.
            except ValueError:
                pass

            # If this move reaches the goal, mark as completed and do not re-add to grid.
            if (nx, ny) == (r.get_goal_x(), r.get_goal_y()):
                r.set_x(nx)
                r.set_y(ny)
                completed_ids.add(rid)
                continue

            # Otherwise, add to the new node and update coordinates.
            # stage the add so all old cells are cleared before any add
            move_plans.append((r, ox, oy, nx, ny))

        # now perform all adds after all removals above
        for r, ox, oy, nx, ny in move_plans:
            if self.nodes[ny][nx].add_robot(r):
                r.set_x(nx)
                r.set_y(ny)
            else:
                # blocked cell; revert to old cell
                self.nodes[oy][ox].add_robot(r)
                r.set_x(ox)
                r.set_y(oy)

        # Recompute distances for remaining robots.
        for r in robots:
            # Skip robots that finished this timestep.
            if r.robot_id in completed_ids:
                continue
            r.distance = r.potential[r.get_y()][r.get_x()]

        # Rebuild the priority queue without completed robots.
        new_q = queue.PriorityQueue()
        for r in robots:
            if r.robot_id not in completed_ids:
                new_q.put(r)
        self.robotQueue = new_q

    # global plan then move tick
    def timestep(self, n):

        # Call collect intentions.
        reservation_dict_primary, reservation_dict_secondary = self.collect_reservations()
        # Resolve conflict.
        winners, losers = self.resolve_conflicts(reservation_dict_primary, reservation_dict_secondary)
        # Apply moves (only with final list).
        self.apply_moves(winners)
        # Visualize to show graph with current time step.
        myGrid.visualize(n + 1)

    # show initial frame and then run timestep until complete
    def run_until_done(self):

        n = 0

        # While robots remain on the grid, advance timesteps.
        self.visualize(n)
        while not self.robotQueue.empty():
            self.timestep(n)
            n += 1

        # Visualize final state after completion.
        myGrid.visualize(n)

    # render visualization and animation
    def visualize(self, n):
        # Clear the previous frame to redraw in the same window.
        plt.clf()
        ax = plt.gca()
        type_color = {'q': 'tab:red', 'd': 'tab:green', 'h': 'tab:blue'}
        type_marker = {'q': 'X', 'd': '^', 'h': 'o'}
        type_label = {'q': 'Quadrotor', 'd': 'DifferentialDrive', 'h': 'Humanoid'}

        # Draw each robot’s current position, goal, and a dashed connector.
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

        # Style axes, legend, and title; keep consistent layout and bounds.
        plt.title("Robot Grid Navigation - timestep " + str(n))  # Added Title - updated with timestep number
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.grid(True)
        plt.xticks(range(self.n))
        plt.yticks(range(self.n))
        ax.set_aspect('equal', adjustable='box')
        plt.tight_layout(rect=[0.0, 0.0, 1.0, 1.0])  # Leave extra space on the right for the legend.
        plt.xlim(-1, self.n)
        plt.ylim(-1, self.n)
        plt.draw()   # Redraw the current figure.
        plt.pause(3) # Keep a 3-second delay between frames.



# receive user input for nxn grid value, then run program
myGrid = Grid(int(input("Enter a grid nxn value. n = ")))
myGrid.run_until_done()
