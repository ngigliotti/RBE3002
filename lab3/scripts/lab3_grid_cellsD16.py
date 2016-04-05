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

    global dummyData
    list1 = []
    for i in range(width*height):
        list1.append(0)
    list1[0] = 100
    dummyData = list1


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
	print 'Staring AStar'
	print 'Width: %d' % width
	print 'Height %d' % height
	print 'Start X: %d' % start[0]
	print 'Start Y: %d' % start[1]
	print 'Goal X: %d' % goal[0]
	print 'Goal Y: %d' % goal[1]

	# Estimates the distance to the goal
	def h(point, goal):
		# Manhattan distance with diagonal movement
		return max(abs(point[0] - goal[0]), abs(point[1] - goal[1]))

	def isWall(point):
		value = mapData[point[0] * (width) + point[1]]
		if value == 100:
			return True
		return False

	visited = []    # The set of nodes already evaluated.
	queue = [(start, 0, [])]            # The set of tentative nodes to be evaluated, initially containing the start node.  The nodes in this set are the nodes that make the frontier between the closed set and all other nodes.
	current = 0

	direction = [(i, j) for i in range(-1, 2) for j in range (-1, 2)]
	direction.remove((0, 0))

	i = 0
	while queue:
		node, cost, path = queue.pop(0)

		if node == goal:
			print 'Found path: ', path
			print 'cost: ', cost
			showCellsVisited(path, visited)
			#for n in path:
				#print isWall(n)
			break

		if node not in visited:
			visited.append(node)

		# if isWall(node):
			# print 'encountered a wall. Darn'
			# continue

		if i == 32:
			i = 0
			showCellsVisited(path, visited)
		else:
			i += 1


		# Expand neighboring nodes in the list
		for d in direction:
			next_node = [node[0] + d[0], node[1] + d[1]]

			if next_node in visited or isWall(next_node):
				continue

			# Check if wall here

			next_cost = len(path) + 1 + h(next_node, goal)

			i = 0
			while i < len(queue):
				if next_cost < queue[i][1]:
					break
				i += 1

			for j in range(len(queue), i, -1):
				if j >= len(queue):
					queue.append(queue[j-1])
				else:
					queue[j] = queue[j-1]
			next_path = path[:]
			next_path.append(next_node)
			next_item = (next_node, next_cost, next_path)
			if next_node not in visited:
				if i >= len(queue):
					queue.append(next_item)
				else:
					queue[i] = next_item
				visited.append(next_node)
'''
def aStar(start,goal):
	print 'Staring AStar'
	print 'Width: %d' % width
	print 'Height %d' % height
	print 'Start X: %d' % start[0]
	print 'Start Y: %d' % start[1]
	print 'Goal X: %d' % goal[0]
	print 'Goal Y: %d' % goal[0]
	closedset = []    # The set of nodes already evaluated.
	openset = [start]            # The set of tentative nodes to be evaluated, initially containing the start node.  The nodes in this set are the nodes that make the frontier between the closed set and all other nodes.
	came_from = [[]]    # The map of navigated nodes.
	for i in range(width):
		list = []
		for j in range(height):
			list.append(None)
		came_from.append(list)
	came_from[start[0]][start[1]] = None
	
	# The g_score of a node is the distance of the shortest path from the start to the node.
	# Start by assuming that all nodes that have yet to be processed cannot be reached 
	g_score = [[]]
	for i in range(width):
		list = []
		for j in range(height):
			list.append(10000000)
		g_score.append(list)

	
	# The starting node has zero distance from start
	g_score[start[0]][start[1]] = 0
	
    # The f_score of a node is the estimated total cost from start to goal to the goal.  This is the sum of the g_score (shortest known path) and the h_score (best possible path).
    # assume same as g_score
	f_score = [[]]
	for i in range(width):
		list = []
		for j in range(height):
			list.append(10000000)
		f_score.append(list)
	# heuristic_cost_estimate(a, b) is the shortest possible path between a and b, this can be euclidean, octodirectional, Manhattan, or something fancy based on how the machine moves
	# the best possible distance between the start and the goal will be the heuristic
	f_score[start[0]][start[1]] = g_score[start[0]][start[1]] + heuristic_cost_estimate(start, goal)
     
	current = 0
	while (len(openset) > 0):
		showCells(closedset)
		print 'Exploring...'                                         # while there are still nodes that have not been checked, continually run the algorithm
		for i in range(len(openset)):
			if f_score[openset[i][0]][openset[i][1]] > current:
				current = [openset[i][0], openset[i][1]]			# this is the most promising node of all nodes in the open set
		if current == goal:                                           # if the best possible path found leads to the goal, it is the best possible path that the robot could discover
			return reconstruct_path(came_from, goal)

		if current in openset:
			openset.remove(current)                  # mark this node as having been evaluated
		closedset.append(current) 

		neighbors = neighbor_nodes(current)
		for i in range(len(neighbors)): 				# re-evaluate each neighboring node
			if neighbors[i] in closedset:
				continue
			tentative_g_score = g_score[current[0]][current[1]] + dist_between(current,neighbors[i])	# create a new g_score for the current neighbor by adding the g_score from the current node and
			                                                                      					# the distance to the neighbor
 
			if neighbors[i] not in openset or tentative_g_score < g_score[neighbors[i][0]][neighbors[i][1]]:                # if the neighbor has not been evaluated yet, or if a better path to the neighbor has been found,
				                                                                                							# update the neighbor
				came_from[neighbors[i][0]][neighbors[i][1]] = current                                                   							# The node to reach this node from in the best time is the current node
				g_score[neighbors[i][0]][neighbors[i][1]] = tentative_g_score                                           							# The G score of the node is what we tentatively calculated earlier
				f_score[neighbors[i][0]][neighbors[i][1]] = g_score[neighbors[i][0]][neighbors[i][1]] + heuristic_cost_estimate(neighbors[i], goal) 							# The F score is the G score and the heuristic
				if neighbors[i] not in openset:                                                      							# add this neighbor to the frontier if it was not in it already
					openset.append(neighbors[i])

	print "There is not a possible path"
'''
	#if the program runs out of nodes to check before it finds the goal, then a solution does not exist

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
    while (came_from[current[0]][current[1]] is not None):
		
		# The current node is now the node that leads to the previous node
        current = came_from[current[0]][current[1]]
		
		# add the current node to the front of the list
        total_path.append(current)
		
	# The list is now the shortest path from the start to the end
    return total_path


def heuristic_cost_estimate(start, goal):
	distance = math.sqrt((goal[0] - start[0])**2 + (goal[1]- start[1])**2)
	return distance


def neighbor_nodes(current):
	nieghbors = []
	nieghbors.append([current[0]+1, current[1]+1])
	nieghbors.append([current[0]+1, current[1]])
	nieghbors.append([current[0]+1, current[1]-1])
	nieghbors.append([current[0]-1, current[1]+1])
	nieghbors.append([current[0]-1, current[1]])
	nieghbors.append([current[0]-1, current[1]-1])
	nieghbors.append([current[0], current[1]+1])
	nieghbors.append([current[0], current[1]-1])
	for i in range(len(nieghbors)):
		pass
	return nieghbors 				#all nodes adjacent to the current node, this could be a list, an array, or any number of existing or custom data-types
	

def dist_between(current,neighbor):
	distance = math.sqrt((neighbor[0] - current[0])**2 + (neighbor[1]- current[1])**2)
	return distance


def publishCells(grid, num):
    global pub
    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for i in range(1,height): #height should be set to hieght of grid
        for j in range(0,width): #width should be set to width of grid
            # print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=(j*resolution)+offsetX + (.5 * resolution) # added secondary offset 
                point.y=(i*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
                point.z=0
                cells.cells.append(point)
            k=k+1
        k=k+1
    if num == 1:
    	pub.publish(cells)
    if num == 2:
    	pubpath.publish(cells)
    if num == 3:
    	pubway.publish(cells)          


def showCells(list):
	grid = []
	for i in range(width + 1):
		for j in range(height + 1):
			grid.append(0)
	for i in range(len(list)):
		grid[(width+1)*list[i][0] + list[i][1]] = 100
	publishCells(grid, 2)

def showCellsVisited(list, visited):
	grid = []
	for i in range(width + 1):
		for j in range(height + 1):
			grid.append(0)
	for i in range(len(visited)):
		grid[(width+1)*visited[i][0] + visited[i][1]] = 100
	for i in range(len(list)):
		grid[(width+1)*list[i][0] + list[i][1]] = 100
	publishCells(grid, 3)


#Main handler of the project
def run():
    global pub
    global pubpath
    global pubway
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
    goal_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)
    start = [20, 20]
    goal = [160, 160]
    # showCells([start, goal])
    #publishCells(mapData, 3)
    #path = aStar(start, goal)
    #showCells(path)
    # while (1 and not rospy.is_shutdown()):
    #     publishCells(dummyData)
    #     publishCells(mapData) #publishing map data every 2 seconds
    #     rospy.sleep(2)  
    #     print("Complete")

    aStar(start,goal)
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
