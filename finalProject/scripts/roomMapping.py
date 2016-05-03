#!/usr/bin/env python

import os
from drive import navToPose, rotate
from settings import MyGlobals
from display import convertToCells, showCells, convertToPose
from geometry_msgs.msg import Point
from pathPlanning import pathPlanningNav
from geometry_msgs.msg import Point
import math, rospy


def mapRoom():
	mapComplete = False
	# rotate in a cricle
	rotate(30)
	rotate(-30)
	rotate(0)
	

	rospy.sleep(10)

	while (not mapComplete):
		goal = findFurthestFrontier()
		if (goal.x<=0 and goal.y<=0):
			mapComplete = True
		goal = convertToPose(goal, MyGlobals.globalMap)
		pathPlanningNav(goal)
	print 'Im Done'
	os.system("spd-say \"I am Done\"")



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
		index = (node.y-1)*width + node.x
		if index < width*height:
			value = mapData[index]
			if value == -1:
				return True
			return False



	def isExplored(node):
		index = (node.y-1)*width + node.x
		if index < width*height:
			value = mapData[index]
			if value == 0:
				return True
			return False



	def findFurthest(list):
		distance = 0
		node = Point()
		currentCell = convertToCells(MyGlobals.robotPose, MyGlobals.globalMap)
		for n in list:
			dist = math.sqrt((n.x - currentCell.x)**2 + (n.y - currentCell.y)**2)
			if (dist > distance):
				distance = dist
				node = n

		return node


	def frontierExpansion(node):
		for d in directions:
			pointTemp = Point()
			pointTemp.x = node.x + 9*d[0]
			pointTemp.y = node.y + 9*d[1]
			if frontierLargeEnough(pointTemp):
				return [pointTemp]
			else:
				continue
		return []




	def frontierLargeEnough(point):
		directions = [(i, j) for i in range(-(robotSize/2), (robotSize/2) + 1) for j in range(-(robotSize/2), (robotSize/2) + 1)]
		directions.remove((0, 0))	
		for d in directions:
			next_node = Point()
			next_node.x = point.x + d[0]
			next_node.y = point.y + d[1]

			if (not isExplored(next_node)):	
				return False
		return True

	if (not isExplored(start)):
		for d in directions:
			newStart = Point()
			newStart.x = start.x + d[0]
			newStart.y = start.y + d[1]
			if isExplored(newStart):
				start = newStart
				showCells([start], MyGlobals.pubStart, MyGlobals.globalMap)
				break


	while queue:
		# Shows Expansion
		if len(visited) % 100 == 0:
			showCells(visited, MyGlobals.pubExplored, MyGlobals.globalMap)

		# Takes first iten of the queue
		node, cost, path = queue.pop(0)

		if isFrontier(node):
			# Sets a new node that the robot can navigate to
			newNode = frontierExpansion(node)
			if newNode:
				print 'Found frontier'
				frontiers.append(newNode[0])
				showCells(frontiers, MyGlobals.pubEnd, MyGlobals.globalMap)
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
	frontier = findFurthest(frontiers)
	showCells([frontier], MyGlobals.pubEnd, MyGlobals.globalMap)

	print 'Waiting...'
	rospy.sleep(10)
	return frontier
