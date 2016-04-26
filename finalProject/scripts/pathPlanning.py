#!/usr/bin/env python

from settings import MyGlobals
from display import convertToCells, convertToPose, showCells, clearAllDisplay
from geometry_msgs.msg import Point, Pose, PointStamped
from drive import navToPose
import math
import rospy
import tf

transform = tf.TransformListener()



# start: Pose object of starting position
# goal: Pose object of goal position
# map: OccupancyGrid object to calculate A* on
def aStar(start, goal, map):
	width = map.info.width
	height = map.info.height
	mapData = map.data

	# Converts to Local Map if needed
	if map == MyGlobals.localMap:
		start.position = transformToLocal(start.position)
		goal.position = transformToLocal(goal.position)

	start = convertToCells(start, map)
	goal = convertToCells(goal, map)

	visited = []
	queue = [(start, 0, [])]
	current = 0

	# 8 directions of move allowed (includes diagnols)
	directions = [(i, j) for i in range(-1, 2) for j in range(-1, 2)]
	directions.remove((0, 0))



	# point: Point object of the current position
	def isWall(point):
		index = point.y*width + point.x
		if index < width*height:
			value = mapData[index]
			if value >= MyGlobals.obstacles:
				return True
			return False



	# point: Point object of the current position
	# goal: Point object of the goal position
	def hueristic(point, goal):
		return math.sqrt((goal.x - point.x)**2 + (goal.y - point.y)**2)



	print 'Starting AStar...'

	# Show Start and End points in RViz
	showCells([start], MyGlobals.pubStart, map)
	showCells([goal], MyGlobals.pubEnd, map)


	while queue:

		# Show Explored Cells
		showCells(visited, MyGlobals.pubExplored, map)

		node, cost, path = queue.pop(0)

		if node == goal:
			print 'Found path'

			# Shows Path
			showCells(path, MyGlobals.pubTotalPath, map)

			# Clears Exploring Area
			showCells([], MyGlobals.pubExplored, map)

			return path
			break

		if node not in visited:
			visited.append(node)

		if isWall(node):
			print 'Encountered a wall. Darn'
			continue

		# Expand neighboring nodes in the list
		for d in directions:
			next_node = Point()
			next_node.x = node.x + d[0]
			next_node.y = node.y + d[1]

			if next_node in visited or isWall(next_node):
				continue

			# Weights diagnol paths 
			if d[0]**2 + d[1]**2 > 1:
				next_cost = len(path) + 1.4 + hueristic(next_node, goal)
			else:
				next_cost = len(path) + 1 + hueristic(next_node, goal)

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



# path: List of Pose objects for the calculated path
def wayPoints(path, map):
	gridCellRes = map.info.resolution #meters per grid cell
	maxDist = 1 #maximum distance before a new waypoint
	points = list()
	straightLine = 0 #distance travelled consecutively in a straight line without a waypoint

	print 'Calculating waypoints...'
	for i  in range(len(path) - 2):
		currentAngle = math.atan2(path[i + 2].y - path[i].y, path[i + 2].x - path[i].x) #find the angle between the current and second next points

		if (i != 0): #if it's not the first point
			straightLine += math.sqrt((path[i].y - path[i - 1].y)**2 + (path[i].x - path[i - 1].x)**2)*gridCellRes

			if ((currentAngle - lastAngle)**2 > 0.15): #if the angle has changed
				points.append(path[i]) #add the point to the list of waypoints
				straightLine = 0

			elif (straightLine >= maxDist): #if the straightLine distance has exceeded the maxDist
				points.append(path[i]) #add the point to the list of waypoints
				straightLine = 0

		lastAngle = currentAngle

	if path:
		points.append(path[-1]) #add the last point to the list of waypoints

		print 'There are %d waypoints' % len(points)

	showCells(points, MyGlobals.pubWaypoints, map)
	return points



# goal: Pose object of goal to plan/navigate to
def pathPlanningNav(goal): 

	goal = goal.pose

	print 'Starting Path Planning...'

	firstLocalWaypoint = MyGlobals.robotPose

	# Loops until the first waypoint in the local path is the goal
	while firstLocalWaypoint.position.x != goal.position.x and firstLocalWaypoint.position.y != goal.position.y:
		print 'Global Map Path Planning...'

		# Calculates overall path on the global map
		globalPath = aStar(MyGlobals.robotPose, goal, MyGlobals.globalMap)
		globalWaypoints = wayPoints(globalPath, MyGlobals.globalMap)
		firstGlobalWaypoint = globalWaypoints[0]
		newGoal = Pose()
		newGoal = convertToPose(firstGlobalWaypoint, MyGlobals.globalMap)

		print 'Local Map Path Planning...'

		# Calculates path to first waypoint of overall path
		localPath = aStar(MyGlobals.robotPose, newGoal, MyGlobals.localMap)
		localWaypoints = wayPoints(localPath, MyGlobals.localMap)
		firstLocalWaypoint = localWaypoints[0]
		localGoal = Pose()
		localGoal.position = firstLocalWaypoint

		# Navigates to first waypoint in the local path
		navToPose(localGoal, False)

	# Navigates to the final position anf orientation
	navToPose(goal, True)

	print 'Done Path Planning'



def transformToGlobal(point):
	pointIn = PointStamped()
	pointIn.point = point
	pointIn.header.frame_id = 'map'
	pointIn.header.stamp = rospy.Time(0)

	pointOut = PointStamped()

	pointOut = transform.transformPoint('odom', pointIn)

	return pointOut.point



def transformToLocal(point):
	pointIn = PointStamped()
	pointIn.point = point
	pointIn.header.frame_id = 'odom'
	pointIn.header.stamp = rospy.Time(0)

	pointOut = PointStamped()

	pointOut = transform.transformPoint('map', pointIn)

	return pointOut.point