import heapq
import sys
import time
from collections import deque
from collections import defaultdict

# Class referenced from the blender build scripts to print solution paths
# in color to the console, makes use of ANSI escape sequences.

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# Re-formats a .txt as a list of lists, used to parse the given maze into a
# format that resembles a 2D matrix.

# Inputs: Filename of maze
# Ouput: Maze in the form of a list of lists

def file2matrix(filename):
    with open(filename, "r") as file:
        temp = file.read().splitlines()
        arr = [list(line) for line in temp]
    return arr

# Finds the co-ordinates of the dots in the maze

# Input: Maze, in the form of a list of lists
# Output: Co-ordinates of dots in the maze, in the form of a list
# Returns empty list if no dots found

def findDots(arg):
    dots = []
    for i, row in enumerate(arg):
        for j, col in enumerate(row):
            if col == ".":
                dots.append((i, j))
    return dots

# Finds the co-ordinate of the start position in the maze

# Input: Maze, in the form of a list of lists
# Output: Co-ordinate of the "P" in the maze, in the form of a tuple
# Returns (None, None) if no start position found

def findStart(arg):
    for i, row in enumerate(arg):
        for j, col in enumerate(row):
            if col == "P":
                return (i, j)
    return (None, None)

# Computes the Manhattan Distance between two nodes

# Inputs: n = co-ordinate of node being explored as a tuple, g = co-ordinate of
# goal node as a tuple
# Output: Manhattan Distance between the two co-ordinates as an int

def manhattanDistance(n, g):
    xn, yn = n
    xg, yg = g
    ret = abs(xn - xg) + abs(yn - yg)
    return ret

# Expands a given node in the maze by discovering it's non-wall children i.e.
# nodes that can be reached by it. Looks in all four directions of the node.
# To be used by search algorithms to determine the next set of nodes to explore.

# Inputs: n = co-ordinate of the node being explored as a tuple, arg = maze as
# a list of lists
# Output: List containing the co-ordinates of all of the given node's children

def expandNode(n, arg):
    children = []
    x, y = n
    up = (x, y+1)
    down = (x, y-1)
    left = (x-1, y)
    right = (x+1, y)

    if maze[up[0]][up[1]] != "%":
        children.append(up)
    if maze[down[0]][down[1]] != "%":
        children.append(down)
    if maze[left[0]][left[1]] != "%":
        children.append(left)
    if maze[right[0]][right[1]] != "%":
        children.append(right)

    return children

# Checks whether given node has been explored before or not by looking it up
# in the set of explored nodes and the frontier set.

# Inputs: Node to be checked as a tuple, Explored set of nodes & Frontier set
# Ouputs: True or False

def checkExp(n, exp, front):
    if n in exp or n in front:
        return True
    else:
        return False

# Wrapper function that checks for the algorithm in the argument and calls the
# appropriate search algorithm on the maze accordingly.

# Inputs: Algorithm as a string and Maze in the form of a list of lists
# Outputs: 'None,' in case of illegal parameters

def solveMaze(arg, algorithm):
    if algorithm == 'bfs':
        return bfs(arg)
    elif algorithm == 'dfs':
        return dfs(arg)
    elif algorithm == 'astar 1.1':
        return astar(arg, 1)
    elif algorithm == 'astar 1.2':
        return astar(arg, 2)
    elif algorithm == 'greedy':
        return greedy(arg)
    else:
        return None

# Follows the same pattern as bfs and dfs but uses heapq functions to insert
# into frontier list as if it was a priority queue. The priority is based off
# the manhattan distance. The node with the smallest manhattan distance in the
# priority queue is selected every time. Based on the version selected by the
# wrapper function, this algorithm makes decisions on whether to work with
# the single dot or multiple dots problem. In the case of the single dot problem,
# it makes use of heuristics (manhattan distance) to find the shortest path.
# In the case of the multiple dots problem, it makes use of precomputed values
# (as a heuristic) to find an optimal solution. It can also be used in accordance
# with bigDots.txt to solve the 1.2 extra credit problem.

# Inputs: Maze as a list of lists.
# Outputs: None

def astar(arg, version):
    count = 0
    startTime = time.time()

    start = findStart(arg)
    goal = findDots(arg)

    dot_distances = {}
    new_goal = list(goal)
    new_goal.append(start)
    new_goal = tuple(new_goal)

    if (len(goal) > 1):

                                                                                                                            # Use astar to pre-compute shortest distance between each goal/start state
        for dot1 in new_goal:
            for dot2 in new_goal:

                if dot1 != dot2:
                    frontier = [(manhattanDistance(dot1, dot2), (dot1))]
                    heapq.heapify(frontier)
                    explored = set()
                    parent = {}
                    cost = {}
                    cost = defaultdict(lambda: 1000000, cost)
                    cost[dot1] = 0

                    while frontier != 0:
                        current = heapq.heappop(frontier)[1]
                        if current == dot2:
                            break
                        if checkExp(current, explored, frontier):
                            continue
                        explored.add(current)
                        children = expandNode(current, arg)
                        for child in children:
                            if cost[current] + 1 >= cost[child]:
                                continue
                            if child not in explored:                                                                       # This prevents a parent in the map from being added as a child
                                parent[child] = current
                            cost[child] = cost[current] + 1
                            heapq.heappush(frontier, (manhattanDistance(child, dot2) + cost[child],
                                                      child))                                                               # Ranking of node is cost of path from start to it, plus manhattan distance?
                            continue
                    count = count + len(explored)
                    shortest_path = getShortestpath(parent, dot1, dot2)
                    cost = 0
                    for point in shortest_path:
                        for i, row in enumerate(arg):
                            for j, col in enumerate(row):
                                if (i, j) == point:
                                    cost = cost + 1

                    dot_distances[dot1, dot2] = cost

    savegoal = len(goal)
    current_pos = start
    overall_cost = 0
    final_path = [start]
    goal_count = {}
    dot_count = 0
    while goal != []:

        current_dist = 100000

        if savegoal == 1:
            for dot in goal:
                if manhattanDistance(current_pos, dot) < current_dist:
                    current_dist = manhattanDistance(current_pos, dot)
                    current_goal = dot
        elif version == 2:
            for dot in goal:
                if dot_distances[current_pos, dot] < current_dist:
                    current_dist = dot_distances[current_pos, dot]
                    current_goal = dot
        else:
            for dot in goal:
                if manhattanDistance(current_pos, dot) < current_dist:
                    current_dist = manhattanDistance(current_pos, dot)
                    current_goal = dot

        final_path.append(current_goal)
        frontier = [(manhattanDistance(current_pos, current_goal), (current_pos))]
        heapq.heapify(frontier)
        explored = set()
        parent = {}
        cost = {}
        cost = defaultdict(lambda: 1000000, cost)
        cost[current_pos] = 0

        while frontier != 0:
            current = heapq.heappop(frontier)[1]
            if current == current_goal:
                break
            if checkExp(current, explored, frontier):
                continue
            explored.add(current)
            children = expandNode(current, arg)
            for child in children:
                if cost[current] + 1 >= cost[child]:
                    continue
                if child not in explored:                                                                                   # This prevents a parent in the map from being added as a child
                    parent[child] = current
                cost[child] = cost[current] + 1
                heapq.heappush(frontier, (manhattanDistance(child, current_goal) + cost[child],
                                          child))                                                                           # Ranking of node is cost of path from start to it, plus manhattan distance?
                continue

        count = count + len(explored)
        shortest_path = getShortestpath(parent, current_pos, current_goal)
        sol_cost = 0
        for point in shortest_path:
            for i, row in enumerate(arg):
                for j, col in enumerate(row):
                    if (i, j) == point:
                        sol_cost = sol_cost + 1
                        arg[i][j] = "*"
        overall_cost = overall_cost + sol_cost
        print "Cost of the path (excluding start & goal points) is", overall_cost
        for i, line in enumerate(arg):
            print(''.join(arg[i]))

        dot_count = dot_count + 1
        goal_count[current_goal] = dot_count
        if goal != []:
            goal.remove(current_goal)
            arg[current_goal[0]][current_goal[1]] = 'P'
            arg[current_pos[0]][current_pos[1]] = ' '

        for y, row in enumerate(arg):
            for x, elt in enumerate(row):
                if elt == '*':
                    arg[y][x] = ' '

        current_pos = current_goal

    endTime = time.time()
    duration = str(endTime - startTime)
    print "Elapsed time in seconds (A*) is", duration
    print "Number of nodes expanded (A*) are", count
    if savegoal > 1:
        print "Shortest path is", final_path
        new_goal = list(new_goal)
        new_goal.remove(start)
        for dot in new_goal:
            if (goal_count[dot] < 10):
                arg[dot[0]][dot[1]] = str(goal_count[dot])
            else:
                arg[dot[0]][dot[1]] = num_letter((goal_count[dot]))
        for i, line in enumerate(arg):
            print(''.join(arg[i]))

# Follows the same pattern as bfs and dfs but uses heapq functions to insert
# into frontier list as if it was a priority queue. The priority is based off
# the manhattan distance. The node with the smallest manhattan distance in the
# priority queue is selected.

# Inputs: Maze as a list of lists.
# Outputs: None

def greedy(arg):
    startTime = time.time()

    start = findStart(arg)
    goal = findDots(arg)
    frontier = [(manhattanDistance(start,goal[0]),(start))]
    heapq.heapify(frontier)
    explored = set()
    parent = {}

    while frontier != 0:
        current = heapq.heappop(frontier)[1]
        if current == goal[0]:
            break
        if checkExp(current, explored, frontier):
            continue
        explored.add(current)
        children = expandNode(current, arg)
        for child in children:
            heapq.heappush(frontier,(manhattanDistance(child,goal[0]),child))
            if child not in explored:                                                                                       # This prevents a parent in the map from being added as a child
                parent[child] = current
            continue

    endTime = time.time()
    duration = str(endTime - startTime)
    print "Elapsed time in seconds (Greedy) is", duration
    print "Number of nodes expanded (Greedy) are", len(explored)
    shortest_path = getShortestpath(parent, start, goal[0])
    drawSolution(maze, shortest_path)


# Backtraces the shortest path in the maze and formats it to a readable list.
# Assumes cost between each node is 1.

# Input: Parent map that contains the parent for each node in the maze as a
# dictionary, start co-ordinate as a tuple and end co-ordinate as a tuple.
# Outputs: Shortest path of co-ordinates in the maze as a list.

def getShortestpath(parent, start, end):
    path = []
    curr = end
    while (curr != start):
        path.append(curr)
        curr = parent[curr]
    path.reverse()
    path.pop()                                                                                                              # Removes dot/end co-ordinate from the list, for improved visibility on graph
    return path

# Performs the BFS search algorithm on the maze.

# Inputs: Maze as a list of lists.
# Outputs: None

def bfs(arg):

    startTime = time.time()

    start = findStart(arg)
    goal = findDots(arg)
    frontier = deque([(start)])
    explored = set()
    parent = {}

    while frontier != 0:
        current = frontier.popleft()                                                                                        # 'popleft()' treats the deque object as a FIFO queue
        if current == goal[0]:
            break
        if checkExp(current, explored, frontier):
            continue
        explored.add(current)
        children = expandNode(current, arg)
        for child in children:
            frontier.append(child)
            if child not in explored:                                                                                       # This prevents a parent in the map from being added as a child
                parent[child] = current
            continue

    endTime = time.time()
    duration = str(endTime - startTime)
    print "Elapsed time in seconds (BFS) is", duration
    print "Number of nodes expanded (BFS) are", len(explored)
    shortest_path = getShortestpath(parent, start, goal[0])
    drawSolution(maze, shortest_path)

# Performs the DFS search algorithm on the maze.

# Inputs: Maze as a list of lists.
# Outputs: None

def dfs(arg):

    startTime = time.time()

    start = findStart(arg)
    goal = findDots(arg)
    frontier = deque([(start)])
    explored = set()
    parent = {}

    while frontier != 0:
        current = frontier.pop()                                                                                            # 'pop()' treats the deque object as a LIFO stack
        if current == goal[0]:
            break
        if checkExp(current, explored, frontier):
            continue
        explored.add(current)
        children = expandNode(current, arg)
        for child in children:
            frontier.append(child)
            if child not in explored:                                                                                       # This prevents a parent in the map from being added as a child
                parent[child] = current
            continue

    endTime = time.time()
    duration = str(endTime - startTime)
    print "Elapsed time in seconds (DFS) is", duration
    print "Number of nodes expanded (DFS) are", len(explored)
    shortest_path = getShortestpath(parent, start, goal[0])
    drawSolution(maze, shortest_path)

# Draws the solution (shortest path in case of BFS/A*) to the maze.

# Inputs: Maze as a list of lists and shortest path co-ordinates in the form of a list.
# Outputs: Draws solution to console and also prints out cost of the path.

def drawSolution(arg, sol):
    cost = 0
    for point in sol:
       for i, row in enumerate(arg):
            for j, col in enumerate(row):
               if (i, j) == point:
                    cost = cost + 1
                    arg[i][j] = bcolors.OKBLUE + "*" + bcolors.ENDC
    print "Cost of the path (excluding start & goal points) is", cost
    for i, line in enumerate(arg):
        print(''.join(arg[i]))

# Outputs the letter corresponding to the input number, this is utilized in 1.2
# while outputting the solution of the search to show the
# numbering of the dots in the order in which they are reached. Letters are used
# as opposed to two digit numbers since, two digit numbers take up more space
# than just one character, moving the maze out of proportion.

# Inputs: Number as an int
# Outputs: Letter as a string

def num_letter(num):
    if (num == 10):
         return 'a'
    if (num == 11):
         return 'b'
    if (num == 12):
         return 'c'
    if (num == 13):
         return 'd'
    if (num == 14):
         return 'e'
    if (num == 15):
         return 'f'
    if (num == 16):
         return 'g'
    if (num == 17):
         return 'h'
    if (num == 18):
         return 'i'
    if (num == 19):
         return 'j'
    if (num == 20):
         return 'k'
    if (num == 21):
         return 'l'
    if (num == 22):
         return 'm'
    if (num == 23):
         return 'n'
    if (num == 24):
         return 'o'
    if (num == 25):
         return 'p'
    if (num == 26):
         return 'q'
    if (num == 27):
         return 'r'
    if (num == 28):
         return 's'
    if (num == 29):
         return 't'
    if (num == 30):
         return 'u'
    if (num == 31):
         return 'v'
    if (num == 32):
         return 'w'
    if (num == 33):
         return 'x'
    if (num == 34):
         return 'y'
    if (num == 35):
         return 'z'

# Test code:

#maze = file2matrix(sys.argv[1])
#result = solveMaze(maze, 'dfs')

#maze = file2matrix(sys.argv[1])
#result = solveMaze(maze, 'bfs')

#maze = file2matrix(sys.argv[1])
#result = solveMaze(maze, 'greedy')

#maze = file2matrix(sys.argv[1])
#result = solveMaze(maze, 'astar 1.1')

maze = file2matrix(sys.argv[1])
result = solveMaze(maze, 'astar 1.2')

# References:
# https://stackoverflow.com/questions/9553638/python-find-the-index-of-an-item-in-a-list-of-lists
# http://book.pythontips.com/en/latest/enumerate.html
# https://stackoverflow.com/questions/2831212/python-sets-vs-lists
# https://stackoverflow.com/questions/8705378/pythons-in-set-operator
# https://stackoverflow.com/questions/12864004/tracing-and-returning-a-path-in-depth-first-search
# https://stackoverflow.com/questions/287871/print-in-terminal-with-colors
