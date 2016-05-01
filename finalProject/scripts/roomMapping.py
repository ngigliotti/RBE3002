#!/usr/bin/env python

from drive import navToPose, rotate
from settings import MyGlobals
from display import convertToCells, showCells, convertToPose
from geometry_msgs.msg import Point
from pathPlanning import pathPlanningNav
from geometry_msgs.msg import Point
import math, rospy


def mapRoom():
	mapComplete = False
	while (not mapComplete):
		goal = findFurthestFrontier()
		goal = convertToPose(goal, MyGlobals.globalMap)
		pathPlanningNav(goal)

		# rotate in a cricle
		#rotate(0)
		#rotate(180)
		#rotate(0)

		mapComplete = checkComlpete()



def findFurthestFrontier():
	print 'Looking for frontier...'

	width = MyGlobals.mainMap.info.width
	height = MyGlobals.mainMap.info.height
	mapData = MyGlobals.mainMap.data
	robotSize = 8

	# 8 directions of move allowed (includes diagnols)
	directions = [(1,0), (0,1), (-1, 0), (0,-1)]

	# Finds Start Node
	start = convertToCells(MyGlobals.robotPose, MyGlobals.mainMap)
	showCells([start], MyGlobals.pubStart, MyGlobals.globalMap)

	# Initializes Lists
	visited = []
	queue = [(start, 0, [])]
	current = 0
	frontiers = []

	# point: Point object of the current position
	def isWall(point):
		index = (point.y-1)*width + point.x
		if index < width*height:
			value = mapData[index]
			if value >= MyGlobals.obstacles:
				return True
			return False



	def isFrontier(node):
		index = node.y*width + node.x
		if index < width*height:
			value = mapData[index]
			if value == -1:
				return True
			return False



	def isExplored(node):
		index = node.y*width + node.x
		if index < width*height:
			value = mapData[index]
			if value == 0:
				return True
			return False



	def findFurthestFrontier(list):
		distance = 0
		node = Point()
		currentCell = convertToCells(MyGlobals.robotPose, MyGlobals.globalMap)
		for n in list:
			dist = math.sqrt((n.x - currentCell.x)**2 + (n.y - currentCell.y)**2)
			if (dist > distance):
				distance = dist
				node = n

		return node



	def frontierLargeEnough(point):
		directions = [(i, j) for i in range(-(robotSize/2), (robotSize/2) + 1) for j in range(-(robotSize/2), (robotSize/2) + 1)]
		directions.remove((0, 0))	
		for d in directions:
			next_node = point
			next_node.x = point.x + d[0]
			next_node.y = point.y + d[1]

			if (not isExplored(next_node)):	
				return False
		return True


	def frontierValid(point):
		directions = [(i, j) for i in range(-1, 2) for j in range(-1, 2)]
		directions.remove((0, 0))

		for d in directions:
			node = Point()
			node.x = point.x + d[0]
			node.y = point.y + d[1]

			if (isExplored(node)):
				i = 0
				while (i <= (robotSize/2)):
					node.x += d[0]
					node.y += d[1]

					i += 1

					if (not isExplored(node)):
						continue
				
				return [node]


			else:
				continue

			return []



	while queue:
		# Shows Expansion
		if len(visited) % 100 == 0:
			showCells(visited, MyGlobals.pubExplored, MyGlobals.globalMap)

		# Takes first iten of the queue
		node, cost, path = queue.pop(0)

		if isFrontier(node):
			if frontierValid(node):
				node = frontierValid(node)[0]
				if frontierLargeEnough(node):
					print 'Found frontier'
					print node
					frontiers.append(node)
					showCells(frontiers, MyGlobals.pubStart, MyGlobals.globalMap)
					rospy.sleep(0.1)
				else:
					print 'Frontier not large enough'
					continue
			else:
				print 'Frontier not valid'
				continue

		if node not in visited:
			visited.append(node)

		for d in directions:
			# Calculates location of next node
			next_node = Point()
			next_node.x = node.x + d[0]
			next_node.y = node.y + d[1]

			if next_node in visited or isWall(next_node):
				continue

			# Calculates Cost
			next_cost = len(path) + 1

			i = 0
			while i < len(queue):
				if next_cost < queue[i][1]:
					break
				i += 1

			for j in range(len(queue), i, -1):
				if j >= len(queue):
					queue.append(queue[j - 1])
				else:
					queue[j] = queue[j - 1]

			next_path = path[:]
			next_path.append(next_node)
			next_item = (next_node, next_cost, next_path)
			if next_node not in visited:
				if i >= len(queue):
					queue.append(next_item)
				else:
					queue[i] = next_item
				visited.append(next_node)

	# Nothing left to search
	frontier = findFurthestFrontier(frontiers)
	print frontiers
	return frontier




def checkComplete():
	return True3