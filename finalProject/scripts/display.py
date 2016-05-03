#!/usr/bin/env python

from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import GridCells
from settings import MyGlobals
import math

# listOfPoints: List of Point objects to show on the rviz map
# pub: ROS Publisher Object of what topic to publish data to
def showCells(listOfPoints, pub, map):

	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = MyGlobals.globalMap.info.resolution
	cells.cell_height = MyGlobals.globalMap.info.resolution
	cells.cells = list()

	for point in listOfPoints:
		point = convertToPose(point, map).position
		cells.cells.append(point)
	pub.publish(cells)



# cellPoint: Point object of grid cell location to convert
# map: OccupancyGrid object of the map to convert on
def convertToPose(cellPoint, map):
	res = map.info.resolution
	xOffset = map.info.origin.position.x
	yOffset = map.info.origin.position.y

	x = cellPoint.x
	y = cellPoint.y

	pose = Pose()

	pose.position.x = (x*res) + xOffset + (0.5*res)
	pose.position.y = (y*res) + yOffset - (0.5*res)

	return pose



# pose: Pose object of position to convert
# OccupancyGrid object of the map to convert on
def convertToCells(pose, map):
	res = map.info.resolution
	xOffset = map.info.origin.position.x
	yOffset = map.info.origin.position.y

	x = pose.position.x
	y = pose.position.y

	point = Point()

	point.x = int((x - 0.5*res - xOffset)/res)
	point.y = int((y + 0.5*res - yOffset)/res)

	return point



# map: OccupancyGrid object of the map to calculate obstacles for
# expansionValue: float of value which determines obstacle
def expandObstacles(map, expansionValue):

	data = map.data
	width = map.info.width

	index = 0
	obstacles = [] 

	for value in data:
		if value >= expansionValue:
			point = Point()
			point.x = index % width
			point.y = math.floor(index/width)

			obstacles.append(point)
		index += 1

	showCells(obstacles, MyGlobals.pubObstacles, map)



def clearAllDisplay():
	showCells ([], MyGlobals.pubExplored, MyGlobals.globalMap)
	showCells ([], MyGlobals.pubTotalPath, MyGlobals.globalMap)
	showCells ([], MyGlobals.pubSubPath, MyGlobals.globalMap)
	showCells ([], MyGlobals.pubWaypoints, MyGlobals.globalMap)