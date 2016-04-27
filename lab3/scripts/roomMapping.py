#!/usr/bin/env python

from drive import navToPose
from settings import MyGlobals

def mapRoom():
	mapComplete = False
	while (! mapNotComplete):
		goal = findNearestFrontier()
		navToPose(goal, False)

		# rotate in a cricle
		rotate(0)
		rotate(180)
		rotate(0)

		mapComplete = checkCompete()



def findNearestFrontier():
	# 8 directions of move allowed (includes diagnols)
	directions = [(i, j) for i in range(-1, 2) for j in range(-1, 2)]
	directions.remove((0, 0))
	start = convertToCells(MyGlobals.robotPose, map)

	visited = []
	queue = [(start, 0, [])]
	current = 0

	while queue:
		if isFrontier(node):
			return node

		if node not in visited:
			visited.append(node)

		for d in directions:
			next_node = Point()
			next_node.x = node.x + d[0]
			next_node.y = node.y + d[1]

			if d[0]**2 + d[1]**2 > 1:
				next_cost = len(path) + 1.4 
			else:
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




def checkComplete():
	return 




def isFrontier(node):
	index = point.y*width + point.x
	if index < width*hegiht:
		value = mapData[index]
		if value == -1:
			return True
		return False
