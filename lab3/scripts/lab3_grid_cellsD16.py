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
                showCells(visited, 3)
		node, cost, path = queue.pop(0)

		if node == goal:
			print 'Found path: ', path
			print 'cost: ', cost
                        showCells(wayPoints(path), 3)
			showCells(path, 2)
                        showCells(wayPoints(path), 3)
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
def wayPoints(path):
    points = list()
    for i in range(len(path)-1):
        current = math.atan2(path[i][1] - path[(i+1)][1], path[i][0] - path[(i+1)][0]) #find the angle between the current and next points
        if (i != 0) and (current != last): #if the angle has changed
            points.append(path[i]) #add the point to the list of waypoints
        last = current
    return points

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


def showCells(list, num):
	grid = []
	for i in range(width + 1):
		for j in range(height + 1):
			grid.append(0)
	for i in range(len(list)):
		grid[(width+1)*list[i][0] + list[i][1]] = 100
	publishCells(grid, num)

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
    goal = [150, 150]
    # showCells([start, goal])
    #publishCells(mapData, 3)
    #path = aStar(start, goal)
    #showCells(path)
    # while (1 and not rospy.is_shutdown()):
    #     publishCells(dummyData)
    #     publishCells(mapData) #publishing map data every 2 seconds
    #     rospy.sleep(2)  
    #     print("Complete")
    showCells([start, goal], 1)
    aStar(start,goal)
    showCells(wayPoints(path), 3)
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
