from enum import Enum
from queue import PriorityQueue
import numpy as np
from shapely.geometry import Polygon, Point, LineString
import sys
import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx
from sklearn.neighbors import KDTree
import numpy.linalg as LA
import scipy

def create_grid_and_polygons(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))
    
    polygons = []        

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        
        obstacle = [north - d_north - safety_distance - north_min, north + d_north + safety_distance - north_min, east - d_east - safety_distance - east_min, east + d_east + safety_distance - east_min]
        #obstacle = [north - d_north - safety_distance, north + d_north + safety_distance, east - d_east - safety_distance, east + d_east + safety_distance]
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]), (obstacle[1], obstacle[2])]
        #corners = [(obstacle[2], obstacle[0]), (obstacle[3], obstacle[0]), (obstacle[3], obstacle[1]), (obstacle[2], obstacle[1])]
        height = alt + d_alt + safety_distance
        p = Polygon(corners)
        polygons.append((p, height))
        
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min), int(north_size), int(east_size), polygons
    
def create_grid_no_alt(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, 0 altitude and safety distance
    arguments to check if entered goal location is in an obstacle
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt > 0:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = alt + d_alt

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    #adding diagonal movements here
    NORTHWEST = (-1, -1, np.sqrt(2))
    NORTHEAST = (-1, 1, np.sqrt(2))
    SOUTHWEST = (1, -1, np.sqrt(2))
    SOUTHEAST = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y, z = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
    #removing diagonal movements here
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTHWEST)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTHEAST)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTHWEST)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTHEAST)
    return valid_actions


def a_star(grid, h, start, goal, TARGET_ALTITUDE):

    TARGET_ALTITUDE = TARGET_ALTITUDE
    
    start3d = (start[0], start[1], TARGET_ALTITUDE)
    goal3d = (goal[0], goal[1], TARGET_ALTITUDE)
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start3d))
    visited = set(start3d)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start3d:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal3d:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1], TARGET_ALTITUDE)
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal3d)
                
                if next_node not in visited:
                    visited.add(next_node)  
                    node3d = (current_node[0], current_node[1], TARGET_ALTITUDE)
                    branch[next_node] = (branch_cost, node3d, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal3d
        path_cost = branch[n][0]
        path.append(goal3d)
        while branch[n][1] != start3d:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost

def a_star_graph(graph, heuristic, start, goal):
    """Modified A* to work with NetworkX graphs."""
    
    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:        
            print('Found a path.')
            #print(goal)
            #print(current_node)
            found = True
            break
        else:
            for next_node in graph[current_node]:
                #print(next_node)
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)
                
                if next_node not in visited:  
                    
                    visited.add(next_node)               
                    queue.put((new_cost, next_node))             
                    branch[next_node] = (new_cost, current_node)
             
    path = []
    path_cost = 0
    if found:
        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        #print(path_cost)
        while branch[n][1] != start:
            #print(branch[n][1])
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')        
    return path[::-1], path_cost

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    #m = np.concatenate((p1, p2, p3), 0)
    m = np.concatenate((p1, p2, p3))
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    pruned_path = [p for p in path]
    
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        # If the 3 points are in a line remove
        # the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if collinearity_check(p1, p2, p3):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path

def check_string_to_float(s):
    try:
        float(s)
        return float(s)
    except:
        return False

"""        
def extract_polygons(data, safety_distance):
    polygons = []
    
    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))
    
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        
        obstacle = [north - d_north - safety_distance - north_min, north + d_north + safety_distance - north_min, east - d_east - safety_distance - east_min, east + d_east + safety_distance - east_min]
        #obstacle = [north - d_north - safety_distance, north + d_north + safety_distance, east - d_east - safety_distance, east + d_east + safety_distance]
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]), (obstacle[1], obstacle[2])]
        #corners = [(obstacle[2], obstacle[0]), (obstacle[3], obstacle[0]), (obstacle[3], obstacle[1]), (obstacle[2], obstacle[1])]
                
        height = alt + d_alt + safety_distance

        p = Polygon(corners)
        polygons.append((p, height))

    return polygons
 """
 
def collides(polygons, point):
    point2d = (point[0], point[1])
    for (p, height) in polygons:
        if p.contains(Point(point2d)) and height > point[2]:
            #print(p, height, point)
            return True
        #print(p, height, point)
    return False
        
def can_connect(n1, n2, polygons):
    n1in2d = (n1[0], n1[1])
    n2in2d = (n2[0], n2[1])
    l = LineString([n1in2d, n2in2d])
    for (p, height) in polygons:
        if p.crosses(l) and height >= min(n1[2], n2[2]):
            return False
    return True

def create_graph(polygones, nodes, k):
    g = nx.Graph()
    tree = KDTree(nodes)
    for n1 in nodes:
        # for each node connect try to connect to k nearest nodes
        idxs = tree.query([n1], k, return_distance=False)[0]
        print("Checking node...")
        for idx in idxs:
            n2 = nodes[idx]
            if n2 == n1:
                continue                
            if can_connect(n1, n2, polygones):
                #add edge and weight is distance between both
                g.add_edge(n1, n2, weight=np.linalg.norm(np.array(n1) - np.array(n2)))
    return g