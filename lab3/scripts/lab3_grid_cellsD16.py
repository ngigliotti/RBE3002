#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math 
import rospy, tf, numpy, math


# reads in global map
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    print data.info

def readGoal(goal):
    global goalX
    global goalY
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
    print goal.pose
    # Start Astar


def readStart(startPos):

    global startPosX
    global startPosY
    startPosX = startPos.pose.pose.position.x
    startPosY = startPos.pose.pose.position.y
    print startPos.pose.pose

def aStar(start,goal):
	closedset = []    # The set of nodes already evaluated.
    openset = [start]            # The set of tentative nodes to be evaluated, initially containing the start node.  The nodes in this set are the nodes that make the frontier between the closed 
	                             # set and all other nodes.
    came_from = the empty map    # The map of navigated nodes.
	
	# The g_score of a node is the distance of the shortest path from the start to the node.
	# Start by assuming that all nodes that have yet to be processed cannot be reached 
    g_score = [][]
    for i in range(width):
    	for j in range(hieght):
    		g_score[i].append(math.inf)

	
	# The starting node has zero distance from start
	readStart(start)
    g_score[startPosX][startPosY] = 0
	
    # The f_score of a node is the estimated total cost from start to goal to the goal.  This is the sum of the g_score (shortest known path) and the h_score (best possible path).
    # assume same as g_score
	f_score = [][]
    for i in range(width):
    	for j in range(hieght):
    		g_score[i].append(math.inf)
	
	# heuristic_cost_estimate(a, b) is the shortest possible path between a and b, this can be euclidean, octodirectional, Manhattan, or something fancy based on how the machine moves
	# the best possible distance between the start and the goal will be the heuristic
    f_score[startPosX][startPosY] = g_score[startPosX][startPosY] + heuristic_cost_estimate(start, goal)
     
	current = 0
    while (not openset):                                         # while there are still nodes that have not been checked, continually run the algorithm
    	for i in range(len(openset)):
    		if f_score[openset[i][0]][openset[i][1]] > current:
    			current = [openset[i][0], openset[i][1]]			# this is the most promising node of all nodes in the open set
        if current = goal                                           # if the best possible path found leads to the goal, it is the best possible path that the robot could discover
            return reconstruct_path(came_from, goal)
         
        openset.remove(current)                  # mark this node as having been evaluated
        closedset.append(current) 

        for each neighbor in neighbor_nodes(current) # re-evaluate each neighboring node
            if neighbor in closedset
                continue
            tentative_g_score = g_score[current] + dist_between(current,neighbor) # create a new g_score for the current neighbor by adding the g_score from the current node and
			                                                                      # the distance to the neighbor
 
            if neighbor not in openset or tentative_g_score < g_score[neighbor]                 # if the neighbor has not been evaluated yet, or if a better path to the neighbor has been found,
				                                                                                # update the neighbor
                came_from[neighbor] = current                                                   # The node to reach this node from in the best time is the current node
                g_score[neighbor] = tentative_g_score                                           # The G score of the node is what we tentatively calculated earlier
                f_score[neighbor] = g_score[neighbor] + heuristic_cost_estimate(neighbor, goal) # The F score is the G score and the heuristic
                if neighbor not in openset                                                      # add this neighbor to the frontier if it was not in it already
                    add neighbor to openset

    print "There is not a possible path"
    return failure #if the program runs out of nodes to check before it finds the goal, then a solution does not exist

# Starting from the goal, work backwards to find the start.  We recommend returning a path nav_msgs, which is an array of PoseStamped with a header
    # create a new instance of the map

    # generate a path to the start and end goals by searching through the neighbors, refer to aStar_explanied.py

    # for each node in the path, process the nodes to generate GridCells and Path messages

    # Publish points

#publishes map to rviz using gridcells type


def reconstruct_path(came_from,current):

	# start by adding goal to the path
    total_path = [current]
	
	# run while reconstruct_path hasn't reached the start
    while (current in came_from):
		
		# The current node is now the node that leads to the previous node
        current = came_from[current]
		
		# add the current node to the front of the list
        total_path.append(current)
		
	# The list is now the shortest path from the start to the end
    return total_path


def heuristic_cost_estimate(start, goal):
	distance = math.sqrt((goal[0] - start[0])**2 + (goal[1]- start[1]**2)
	return distance


def neighbor_nodes(current):
	nieghbors = []

	return #all nodes adjacent to the current node, this could be a list, an array, or any number of existing or custom data-types
	

def dist_between(current,neighbor):
	distance = math.sqrt((nieghbor[0] - current[0])**2 + (nieghbor[1]- current[1]**2)
	return distance


def publishCells(grid):
    global pub
    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for i in range(1,height): #height should be set to hieght of grid
        k=k+1
        for j in range(1,width): #width should be set to width of grid
            k=k+1
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=(j*resolution)+offsetX + (.5 * resolution) # added secondary offset 
                point.y=(i*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
                point.z=0
                cells.cells.append(point)
    pub.publish(cells)           

#Main handler of the project
def run():
    global pub
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
    goal_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)



    while (1 and not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        rospy.sleep(2)  
        print("Complete")
    


if __name__ == '__main__':
    try:
        run()
        readStart()
        readGoal()
        aStar([startPosX, startPosY],[goalX, goalY])
    except rospy.ROSInterruptException:
        pass
