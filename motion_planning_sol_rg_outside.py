import argparse
import time
import msgpack
#import scipy
import networkx as nx
from enum import Enum, auto

import numpy as np

from planning_utils import *
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from shapely.geometry import Polygon, Point, LineString
import numpy.linalg as LA
from sklearn.neighbors import KDTree


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            #changed deadband to 5m for earlier transition. Parameter could be optimized as well
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 5.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        #TARGET_ALTITUDE = 15
        #SAFETY_DISTANCE = 10

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        # Done here
        with open('colliders.csv') as f:
            startPosFromFile = f.readline()
        #print(startPosFromFile)
        startPosFromFileSplit = startPosFromFile.split(',')
        startPosLatSplit = startPosFromFileSplit[0].split()
        startPosLonSplit = startPosFromFileSplit[1].split()
        lat = float(startPosLatSplit[1])
        lon = float(startPosLonSplit[1])
        
        # TODO: set home position to (lon0, lat0, 0)
        # Done here. But this statement is not visible (i.e print home_position will give different result) until getting out of the function when home location is updated
        self.set_home_position(lon, lat, 0.0)

        # TODO: retrieve current global position
        # Done here
        global_pos_current = [self.global_position[0], self.global_position[1], self.global_position[2]]
        #print(global_pos_current)
        
        # TODO: convert to current local position using global_to_local()
        # Done here. Should be close to 0,0,0 as home is equal to initial position
        local_pos_current = global_to_local(global_pos_current, self.global_home)
        #print(local_pos_current)
        
        #global_home is not displayed correctly here. Only after getting out of this function
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
                       
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        
        # Define starting point on the grid (this is just grid center)
        #grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        # Done here. Calculate closest point on grid from local position.
        grid_start = (int(np.around(self.local_position[0]))-north_offset, int(np.around(self.local_position[1]))-east_offset) 
        #get closest point on the graph
        pointStart = [grid_start[0], grid_start[1], TARGET_ALTITUDE] 
        
        #print(distStart)
        #print (indexStart)
        #print(list(nodes_graph)[indexStart])
        
        # TODO: adapt to set goal as latitude / longitude position and convert
        # Done here with user input
        #37.795567 -122.394383
        #37.793493 -122.398230

        # Code for user input. Either use this or set an goal position below
        #""" comment out here to set goal manually in line 219
        validGoal = False
        while validGoal == False:
            print("Please enter longitude and latitude seperately in decimal degrees (e.g. -122.398230, 37.793493)")
            lonValid = False
            while lonValid == False:
                goalLonStr = (input("Longitude?: "))
                goalLon = check_string_to_float(goalLonStr)
                if (goalLon != False and goalLon < 180.0 and goalLon > -180.0):
                    lonValid = True
                else:
                    print("Longitude not valid!")
            latValid = False
            while latValid == False:
                goalLatStr = input("Latitude?: ")
                goalLat = check_string_to_float(goalLatStr)
                if (goalLat != False and goalLat < 90.0 and goalLat > -90.0):
                    latValid = True
                else:
                    print("Latitude not valid!")
            global_pos_goal = [goalLon, goalLat, 0.0]
            #global_pos_goal = [-122.396845, 37.792392, 0.0]
            # Convert to current local position using global_to_local()
            local_pos_goal = global_to_local(global_pos_goal, self.global_home)
            if (np.around(local_pos_goal[0]) >= north_offset and np.around(local_pos_goal[0]) <= north_size and np.around(local_pos_goal[1]) >= east_offset and np.around(local_pos_goal[1]) <= east_size):
                grid_goal = (int(np.around(local_pos_goal[0]))-north_offset, int(np.around(local_pos_goal[1]))-east_offset)
                grid_goal_north = int(np.around(local_pos_goal[0]))-north_offset
                grid_goal_east = int(np.around(local_pos_goal[1]))-east_offset
            
                #print(grid_no_alt[grid_goal[0], grid_goal[1]])
            
                if(grid_no_alt[grid_goal[0], grid_goal[1]] == 0): 
                    print("Goal is set to {0} {1}".format(goalLon, goalLat))
                    validGoal = True
                else:
                    print("Obstacle at goal location!")
            else:
                print("point not on grid")
        #"""
                
        # set goal if not using user input
        #global_pos_goal = [-122.398230, 37.793493, 0.0]
        #global_pos_goal = [-122.393214, 37.795414, 0.0]
        
        # Convert to current local position using global_to_local()
        local_pos_goal = global_to_local(global_pos_goal, self.global_home)
        grid_goal = (int(np.around(local_pos_goal[0]))-north_offset, int(np.around(local_pos_goal[1]))-east_offset)
        
        #get closest point on the graph
        pointGoal = [grid_goal[0], grid_goal[1], TARGET_ALTITUDE]

        #print(distGoal)
        #print (indexGoal)
        #print(list(nodes_graph)[indexGoal])
        
        """
        #get the minimum and maximum values for sampling
        ymin = min(pointStart[0], pointGoal[0])
        ymax = max(pointStart[0], pointGoal[0])
        xmin = min(pointStart[1], pointGoal[1])
        xmax = max(pointStart[1], pointGoal[1])
        
        #sample random points in 2D, let z at TARGET_ALTITUDE and the safety margin
        xvals = np.random.uniform(xmin - xmin, xmax - xmin, num_samples)
        yvals = np.random.uniform(ymin - ymin, ymax - ymin, num_samples)
        zvals = np.empty(num_samples)
        zvals.fill(TARGET_ALTITUDE)
        samples = np.array(list(zip(yvals, xvals, zvals)))
  
        #check nodes for obstacles
        nodes = []
        for point in samples:
            print("Check collides...")
            if not collides(polygons, point):
                nodes.append(tuple(point))
        print(len(nodes))

        g = create_graph(polygons, nodes, NUM_EDGES)

        #create a tree to search for nearest starting point
        nodes_graph = g.nodes
        #print(nodes_graph)
    
        mytree = scipy.spatial.KDTree(nodes_graph)
        """
        
        distStart, indexStart = mytree.query(pointStart)
        distGoal, indexGoal = mytree.query(pointGoal, 3)
        #print(indexGoal)
        
        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        # Done in planning_utils
        print('Local Start and Goal: ', grid_start, grid_goal)
        
        
        print("Calculating path...")
        graphStart = ( int(np.around(list(nodes_graph)[indexStart][0])), int(np.around(list(nodes_graph)[indexStart][1])))
        #print(graphStart)
                
        print("Calculating path to graph...")
        path_to_start, _ = a_star(grid, heuristic, grid_start, graphStart, TARGET_ALTITUDE)
        #print(path_to_start)
        
        print("Calculating path in graph...")
        i = 0
        while i < len(indexGoal):
            testIndex = indexGoal[i]
            #print(testIndex)
            graphGoal = (int(np.around(list(nodes_graph)[testIndex][0])), int(np.around(list(nodes_graph)[testIndex][1])))
            #print(graphGoal)
            path, _ = a_star_graph(g, heuristic, list(nodes_graph)[indexStart], list(nodes_graph)[testIndex])
            i = i+1
            if path == []:
                print("Node not connected to start, trying another node")
                continue
            else:
                break
        if path == []:
            print("No path in graph, using grid... Get a coffee...")
            path, _ = a_star(grid, heuristic, graphStart, graphGoal, TARGET_ALTITUDE)
        #print(path)
        print("Calculating path from graph...")
        path_to_goal, _ = a_star(grid, heuristic, graphGoal, grid_goal, TARGET_ALTITUDE)
        #print(path_to_goal)
        
        path_complete = path_to_start + path + path_to_goal
        
        # TODO: prune path to minimize number of waypoints
        # Done here in planning_utils
        print("Pruning path...")
        pruned_path = prune_path(path_complete)
        #print(pruned_path)
        
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        
        #plot results if you have time
        """
        import matplotlib.pyplot as plt

        plt.imshow(grid, origin='lower', cmap='Greys')

        for i in range(len(pruned_path) - 1):
            p1 = pruned_path[i]
            p2 = pruned_path[i + 1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')

        plt.plot(grid_start[1], grid_start[0], 'gx')
        plt.plot(grid_goal[1], grid_goal[0], 'gx')
    
        # draw edges
        for (n1, n2) in g.edges:
            plt.plot([n1[1], n2[1]], [n1[0], n2[0]], 'black' , alpha=0.5)

        # draw connected nodes
        for n1 in g.nodes:
            plt.scatter(n1[1], n1[0], c='red')

        plt.xlabel('EAST', fontsize=20)
        plt.ylabel('NORTH', fontsize=20)
        plt.show()
        """
        
        # Convert path to 
        # Need the integer version for the visualization
        waypoints = np.array([])
        waypoints_int = np.array([])
        waypoints = []
        waypoints_int = []
        i = 0
        # Saving path in waypoints with heading towards the next waypoint, except for start and goal
        for p in pruned_path:
            if(i+1 == len(pruned_path) or i==0):
                waypoint = []
                waypoint.append (p[0] + north_offset)
                waypoint.append (p[1] + east_offset)
                waypoint.append (TARGET_ALTITUDE)
                waypoint.append (0)
                waypoints.append(waypoint)
                
                waypoint_int = []
                waypoint_int.append (int(np.around(p[0])) + north_offset)
                waypoint_int.append (int(np.around(p[1])) + east_offset)
                waypoint_int.append (TARGET_ALTITUDE)
                waypoint_int.append (0)
                waypoints_int.append(waypoint_int)
                
            else:
                waypoint = []
                waypoint.append (p[0] + north_offset)
                waypoint.append (p[1] + east_offset)
                waypoint.append (TARGET_ALTITUDE)
                waypoint.append (np.arctan2( ( p[1] - pruned_path[i-1][1] ), ( p[0] - pruned_path[i-1][0] )))
                waypoints.append(waypoint)
                
                waypoint_int = []
                waypoint_int.append (int(np.around(p[0])) + north_offset)
                waypoint_int.append (int(np.around(p[1])) + east_offset)
                waypoint_int.append (TARGET_ALTITUDE)
                waypoint_int.append (np.arctan2( ( p[1] - pruned_path[i-1][1] ), ( p[0] - pruned_path[i-1][0] ) ))
                waypoints_int.append(waypoint_int)
                
            i = i+1        
        
        # Set self.waypoints
        # Have to send integer waypoints for the visualization, it fails with floats
        self.waypoints = waypoints_int
        
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()
        
        # Change back to original floats
        self.waypoints = waypoints
        
    def start(self):
        self.start_log("Logs", "NavLog.txt")
        
        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()
    
    TARGET_ALTITUDE = 25
    SAFETY_DISTANCE = 7
    NUM_EDGES = 10
    num_samples = 250
        
    #Putting code here to reduce coputation time here
    
    # Read in obstacle map
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
    
    # Define a grid for a particular altitude and safety margin around obstacles
    grid, north_offset, east_offset, north_size, east_size, polygons = create_grid_and_polygons(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
    # Define a grid with 0 altitude in order to determine if an obsticle exists at entered goal location
    grid_no_alt, north_offset, east_offset = create_grid_no_alt(data, 0, SAFETY_DISTANCE)
               
    #get the minimum and maximum values for sampling
    ymin = north_offset
    ymax = np.max(data[:, 0] + data[:, 3])
    xmin = east_offset
    xmax = np.max(data[:, 1] + data[:, 4])
        
    #sample random points in 2D, let z at TARGET_ALTITUDE and the safety margin
    xvals = np.random.uniform(xmin - xmin, xmax - xmin, num_samples)
    yvals = np.random.uniform(ymin - ymin, ymax - ymin, num_samples)
    zvals = np.empty(num_samples)
    zvals.fill(TARGET_ALTITUDE)
    samples = np.array(list(zip(yvals, xvals, zvals)))
  
    #check nodes for obstacles
    nodes = []
    for point in samples:
        print("Check collides...")
        if not collides(polygons, point):
            nodes.append(tuple(point))
    print(len(nodes))

    g = create_graph(polygons, nodes, NUM_EDGES)

    #create a tree to search for nearest starting point
    nodes_graph = g.nodes
    #print(nodes_graph)
    
    mytree = scipy.spatial.KDTree(nodes_graph)
    

    #print resulting graph
    """
    import matplotlib.pyplot as plt
    plt.imshow(grid, origin='lower', cmap='Greys')

    # draw edges
    for (n1, n2) in g.edges:
        plt.plot([n1[1], n2[1]], [n1[0], n2[0]], 'black' , alpha=0.5)

    # draw all samples
    for n3 in samples:
        plt.scatter(n3[1], n3[0], c='green')

    # draw all nodes
    for n2 in nodes:
        plt.scatter(n2[1], n2[0], c='yellow')

    # draw connected nodes
    for n1 in g.nodes:
        plt.scatter(n1[1], n1[0], c='red')
          
    #for (p, height) in polygons:
    #    plt.scatter(p.bounds[0], p.bounds[1], c='blue')
    #    plt.scatter(p.bounds[2], p.bounds[1], c='blue')
    #    plt.scatter(p.bounds[2], p.bounds[3], c='blue')
    #    plt.scatter(p.bounds[0], p.bounds[3], c='blue')
    
    plt.xlabel('EAST', fontsize=20)
    plt.ylabel('NORTH', fontsize=20)
    plt.show()
    """
    
    #had to put timeout to 1200
    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=1200)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
