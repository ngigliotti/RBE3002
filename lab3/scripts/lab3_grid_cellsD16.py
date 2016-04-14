#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
import tf
import numpy
import math 
import rospy, tf, numpy, math


# Publishes a Twist Message for Driving the Robot
def publishTwist(lin_Vel, ang_Vel):
    """Send a movement (twist) message."""
    global pubmotion
    msg = Twist()
    msg.linear.x = lin_Vel
    msg.angular.z = ang_Vel
    pubmotion.publish(msg)


# reads in local map
def localMapCallBack(data):
	global localMap
	global localMapData
	global localOffsetX
	global localOffsetY
	global localWidth
	global localHeight
	global localResolution

	localMap = data
	localMapData = list(data.data)
	localOffsetX = data.info.origin.position.x
	localOffsetY = data.info.origin.position.y
	localWidth = data.info.width
	localHeight = data.info.height
	localResolution = data.info.resolution

	#print 'Local Map Updated'


# reads in global map
def globalMapCallBack(data):
	global globalMap
	global globalMapData
	global globalOffsetX
	global globalOffsetY
	global globalWidth
	global globalHeight
	global globalResolution

	globalMap = data
	globalMapData = list(data.data)
	globalOffsetX = data.info.origin.position.x
	globalOffsetY = data.info.origin.position.y
	globalWidth = data.info.width
	globalHeight = data.info.height
	globalResolution = data.info.resolution

	#print 'Global Map Updated'


def updateLocal(data):
	global localMap
	global localMapData
	global localOffsetX
	global localOffsetY
	global localWidth
	global localHeight

	#print 'Updating Local Map...'
	update = data.data
	w = data.width
	h = data.height
	x = data.x
	y = data.y

	indexUpdate = 0
	index = x + localWidth*(y-1) 
	for i in range(h):
		for j in range(w):
			localMapData[index] = update[indexUpdate]
			indexUpdate += 1
			index += 1
		index += localWidth - w



def updateGlobal(data):
	global globalMap
	global globalMapData
	global globalOffsetX
	global globalOffsetY
	global globalWidth
	global globalHeight

	#print 'Updating Global Map...'

	update = data.data
	w = data.width
	h = data.height
	x = data.x
	y = data.y

	indexUpdate = 0
	index = x + globalWidth*(y-1) 
	for i in range(h):
		for j in range(w):
			globalMapData[index] = update[indexUpdate]
			indexUpdate += 1
			index += 1
		index += localWidth - w




#keeps track of current location and orientation
def tCallback(event):
    global pose
    global xPosition
    global yPosition
    global theta

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(3.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    pose.position.x=position[0]
    pose.position.y=position[1]
    # the previous 2 lines and next 2 lines are repedative. Joes bad
    xPosition=position[0]
    yPosition=position[1]

    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)

    #convert yaw to degrees
    pose.orientation.z = yaw
    theta = math.degrees(yaw)


def readGoal(goal):
	global goalX
	global goalY
	global desiredT
	global globalMap
	global mapgrid
	global globalResolution
	global globalMapData
	global globalWidth
	global globalHeight
	global globalOffsetX
	global globalOffsetY

	# Intializes on the global Map
	data = globalMap
	mapgrid = data
	globalResolution = data.info.resolution
	globalMapData = list(data.data)
	globalWidth = data.info.width
	globalHeight = data.info.height
	globalOffsetX = data.info.origin.position.x
	globalOffsetY = data.info.origin.position.y

	goalX = goal.pose.position.x
	goalY = goal.pose.position.y
	quat = goal.pose.orientation
	q = [quat.x, quat.y, quat.z, quat.w]
	roll, pitch, yaw = euler_from_quaternion(q)
	desiredT = yaw * (180/math.pi)

	goalX = int((goalX - 0.5*globalResolution - globalOffsetX)/globalResolution)
	goalY = int((goalY + 0.5*globalResolution - globalOffsetY)/globalResolution)

	run()


def transformToGlobal(coord):
	global odom_list
	x = coord[0]
	y = coord[1]
	pointStampedIn = PointStamped()
	pointStampedIn.point.x = x
	pointStampedIn.point.y = y
	pointStampedIn.point.z = 0
	pointStampedIn.header.frame_id = 'odom'
	pointStampedIn.header.stamp = rospy.Time(0)

	pointStampedOut = PointStamped()
	
	pointStampedOut = odom_list.transformPoint('map', pointStampedIn)

	return [pointStampedOut.point.x, pointStampedOut.point.y]


def transformToLocal(coord):
	global mapgrid
	global localResolution
	global localMapData
	global localWidth
	global localHeight
	global localOffsetX
	global localOffsetY
	global localMap
	# Intializes on the global Map
	data = localMap
	mapgrid = data
	localResolution = data.info.resolution
	localMapData = list(data.data)
	localWidth = data.info.width
	localHeight = data.info.height
	localOffsetX = data.info.origin.position.x
	localOffsetY = data.info.origin.position.y


	global odom_list
	x = coord[0]
	y = coord[1]
	pointStampedIn = PointStamped()
	pointStampedIn.point.x = x
	pointStampedIn.point.y = y
	pointStampedIn.point.z = 0
	pointStampedIn.header.frame_id = 'map'
	pointStampedIn.header.stamp = rospy.Time(0)

	pointStampedOut = PointStamped()
	
	pointStampedOut = odom_list.transformPoint('odom', pointStampedIn)

	return [pointStampedOut.point.x, pointStampedOut.point.y]


def navToPose(goalX, goalY):
	global xPosition
	global yPosition
	global theta
	global localMap
	global path
	global localResolution
	global localOffsetX
	global localOffsetY

	print 'Navigating to pose...'

	#capture desired x and y positions
	desiredY = goalY
	desiredX = goalX

    #compute angle required to make straight-line move to desired pose
	angle = math.degrees(math.atan2(desiredY - yPosition, desiredX - xPosition))
	#compute distance to target
	print [xPosition, yPosition]
	print [desiredX, desiredY]
	distance = math.sqrt((desiredX - xPosition)**2 + (desiredY - yPosition)**2)
	print 'Point Destination: ', [desiredX, desiredY]

	rospy.sleep(5)

	print 'Spinning to Angle: %d' % angle #turn to calculated angle
	rotate(0.35, angle)

	#rospy.sleep(10)

	# Transforms start and end points to the LocalMap
	showCells2([[xPosition, yPosition], [desiredX, desiredY]], 4)

	start = transformToLocal([xPosition, yPosition])
	xPos = int((start[0] - 0.5*localResolution - localOffsetX)/localResolution)
	yPos = int((start[1] + 0.5*localResolution - localOffsetY)/localResolution)
	start = [xPos, yPos]

	goal = transformToLocal([desiredX, desiredY])
	desiredX = int((goal[0] - 0.5*localResolution - localOffsetX)/localResolution)
	desiredY = int((goal[1] + 0.5*localResolution - localOffsetY)/localResolution)
	goal = [desiredX, desiredY]

	# Calculate A* on the local map
	print 'Calculating AStar on Local Map...'
	print [start, goal]
	aStar(start, goal, localMap) # Calculates A* on the local map from start to first waypoint
	# Transforms each of the points in the path to Global Space
	rospy.sleep(0.1)
	tempWay = wayPoints(path)

	#print path

	for j in range(len(path)):
		path[j][0] = ((path[j][0])*localResolution)+localOffsetX + (.5 * localResolution)
		path[j][1] = ((path[j][1])*localResolution)+localOffsetY - (.5 * localResolution)
		path[j] = transformToGlobal(path[j])

	tempWay = wayPoints(path)

	showCells2(path, 2)
	if (len(path) > 2):
		showCells2([path[0], path[-1]], 4)

	#print 'waypoints...'
	#print tempWay
	showCells2(tempWay, 3)
	#print tempWay
	
	# New Values after calculating on local map
	desiredY = tempWay[0][1]
	desiredX = tempWay[0][0]
	print [desiredX, desiredY]
	print [xPosition, yPosition]


	#if ((tempWay[0][0]-path[-1][0])**2 + (tempWay[0][1]-path[-1][1])**2 > .5):
	#	print 'Obstacle Found!'
	#	print (tempWay[0][0]-path[-1][0])**2 + (tempWay[0][1]-path[-1][1])**2
	#compute angle required to make straight-line move to desired pose
	angle = math.degrees(math.atan2(desiredY - yPosition, desiredX - xPosition))
	#compute distance to target
	distance = math.sqrt((desiredX - xPosition)**2 + (desiredY - yPosition)**2)

	#rospy.sleep(5)

	#print [desiredX, desiredY]
	print 'Spinning to Angle: %d' % angle #turn to calculated angle
	rotate(0.35, angle)

	print 'Moving Distance: ', distance #move in straight line specified distance to new pose
	driveStraight(0.15, distance)


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    """This function accepts a speed and a distance for the robot to move in a straight line"""
    global pose

    initialX = pose.position.x
    initialY = pose.position.y
    atTarget = False

    #Loop until the distance between the attached frame and the origin is equal to the
    #distance specified 
    while (not atTarget and not rospy.is_shutdown()):
        currentX = pose.position.x
        currentY = pose.position.y
        currentDistance = math.sqrt((currentX-initialX)**2 + (currentY-initialY)**2) #Distance formula
        if (currentDistance >= distance):
            atTarget = True
            publishTwist(0, 0)

        else:
            publishTwist(speed, 0)
            rospy.sleep(0.05)


def rotate(speed, angle): 
    global odom_list
    global pose
    vel = Twist();   
    direction = 1

    # set rotation direction
    error = angle-math.degrees(pose.orientation.z)
    if (error > -180 and error < 0):
        direction = -1

    # Turns until getting the error gets small
    while ((abs(error) >= 2) and not rospy.is_shutdown()):
        publishTwist(0, direction*speed)
        rospy.sleep(0.05)    
        #print "theta: %d" % math.degrees(pose.orientation.z)
        error = angle-math.degrees(pose.orientation.z)   
    vel.angular.z = 0.0
    pubmotion.publish(vel)



def aStar(start, goal, data):
	global localMap
	global cost
	global path
	global obstacles
	mapgrid = data
	resolution = data.info.resolution
	mapData = list(data.data)
	width = data.info.width
	height = data.info.height
	offsetX = data.info.origin.position.x
	offsetY = data.info.origin.position.y

	expandedBarrier = 60

	# Estimates the distance to the goal
	def h(point, goal):
		# Manhattan distance with diagonal movement
		#return max(abs(goal[0] - point[0]), abs(goal[1] - point[1]))
		return math.sqrt((goal[0] - point[0])**2 + (goal[1]- point[1])**2)

	def isWall(point):
		index = point[1] * (width) + point[0]
		if (index < width * height):
			value = mapData[index]
			if value >= expandedBarrier:
				return True	
			return False		#Else
		else:
			print 'No Valid Path'
			return [[]]

	visited = []    				# The set of nodes already evaluated.
	queue = [(start, 0, [])]		# The set of tentative nodes to be evaluated, initially containing the start node.
	current = 0

	direction = [(i, j) for i in range(-1, 2) for j in range (-1, 2)]
	direction.remove((0, 0))

	i = 0
	while queue:

		#showCells(visited, 5)

		node, cost, path = queue.pop(0)

		if node == goal:
			print 'Found path'
			print 'cost: ', cost
			break

		if node not in visited:
			visited.append(node)

		if isWall(node):
			if (node == start):
				expandedBarrier = mapData[node[1] * (width) + node[0]]
			if (node == goal):
				expandedBarrier = mapData[node[1] * (width) + node[0]]
			else:
				print 'Encountered a wall. Darn'
				continue

		# Expand neighboring nodes in the list
		for d in direction:
			next_node = [node[0] + d[0], node[1] + d[1]]

			if next_node in visited or isWall(next_node):
				continue

			# Check if wall here
			if (d[0]**2 + d[1]**2 > 1):		# If Diagnol Direction
				next_cost = len(path) + 5 + h(next_node, goal)
			else:
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
	gridCellRes = 0.2 #inches per grid cell
	maxDist = 4 #maximum distance before a new waypoint
	points = list()
	straightLine = 0 #distance travelled consecutively in a straight line without a waypoint

	for i in range(len(path)-1):
		currentAngle = math.atan2(path[(i+1)][1] - path[i][1], path[(i+1)][0] - path[i][0]) #find the angle between the current and next points

		if (i != 0): #if it's not the first point
			straightLine += math.sqrt((path[i][1] - path[(i-1)][1])**2 + (path[i][0] - path[(i-1)][0])**2) * gridCellRes 

			if ((currentAngle-lastAngle)**2 > .15): #if the angle has changed
				#path[i].append(currentAngle) #add the current angle to the coordinates 
				points.append(path[i]) #add the point to the list of waypoints
				straightLine = 0

			elif (straightLine >= maxDist): #if the straightLine distance has exceeded the maxDist
				#path[i].append(currentAngle) #add the current angle to the coordinates
				points.append(path[i]) #add the point to the list of waypoints
				straightLine = 0

		lastAngle = currentAngle
	#path[len(path) - 1].append(currentAngle) #add the angle to the coordinates of the last point
	if path:
		points.append(path[-1]) #add the last point to the list of waypoints

	return points


def wayPoints2(path):
	points = list()
	maxDist = 5
	i = 0


	while (i < len(path) - 1):
		if (i + maxDist > len(path)):
			points.append(path[-1])
			return points
		i+=maxDist
		points.append(path[i])

	return points



def moveWithAStar(start, goal):
	global xPosition
	global yPosition
	global path
	global globalResolution
	global globalOffsetX
	global globalOffsetY
	global desiredT
	global cost
	global globalMap
	global localMap

	showCells([[0, 0]], 1)
	showCells([[0, 0]], 2)
	showCells([[0, 0]], 3)
	showCells([[0, 0]], 4)
	showCells([[0, 0]], 5)
	showCells([start,goal], 4)

	print 'Calculating AStar on Global Map...'
	print [start, goal]
	aStar(start, goal, globalMap)
	way = wayPoints(path)
	showCells(path, 2)
	showCells(way, 3)

	while len(way) > 1 and len(path) > 2:
		wayX = ((way[0][0])*globalResolution)+globalOffsetX + (.5 * globalResolution)
		wayY = ((way[0][1])*globalResolution)+globalOffsetY - (.5 * globalResolution)
		navToPose(wayX, wayY)

		# Updates Starting Position
		xPos = int((xPosition - 0.5*globalResolution - globalOffsetX)/globalResolution)
		yPos = int((yPosition + 0.5*globalResolution - globalOffsetY)/globalResolution)
		start = [xPos, yPos]

		# Calculates A* on the Global map from current waypoint to goal
		print 'Calculating AStar on Global Map...'
		print [start, goal]
		aStar(start, goal, globalMap)
		way = wayPoints(path)
		showCells(path, 2)
		showCells(way, 3)

	goalX = (goal[0]*globalResolution)+globalOffsetX + (.5 * globalResolution)
	goalY = (goal[1]*globalResolution)+globalOffsetY - (.5 * globalResolution)
	print 'Final Time-----------------------------------'
	print [goalX, goalY]
	navToPose(goalX, goalY)
	print 'Rotating to Final Orientation'
	rotate(.35, desiredT)
	print 'Arrived at Goal'



def publishCells(grid, num):
    global pub
    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for i in range(1, height): #height should be set to hieght of grid
        k=k+1
        for j in range(1, width): #width should be set to width of grid
            k=k+1
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=(j*resolution)+offsetX + (.5 * resolution) # added secondary offset 
                point.y=(i*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
                point.z=0
                cells.cells.append(point)
    pub.publish(cells)       



def showCells(list, num):
	global pub
	global pubpath
	global pubway
	global pubpoints
	global pubexplore

	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = globalResolution 
	cells.cell_height = globalResolution

	for i in range(len(list)): 
		point=Point()
		#print list[i]
		point.x=(list[i][0]*globalResolution)+globalOffsetX + (.5 * globalResolution) # added secondary offset 
		point.y=(list[i][1]*globalResolution)+globalOffsetY - (.5 * globalResolution) # added secondary offset ... Magic ?
		if num == 1:
			point.z = 0
		if num == 2:
			point.z = .25
		if num == 3:
			point.z = 0.5
		if num == 4:
			point.z = 0.75
		if num == 5:
			point.z = .2
		cells.cells.append(point)
		#print point

	if num == 1:
		pub.publish(cells)
	if num == 2:
		pubpath.publish(cells)
	if num == 3:
		pubway.publish(cells) 
	if num == 4:
		pubpoints.publish(cells)
	if num == 5:
		pubexplore.publish(cells)



def showCells2(list, num):
	global pub
	global pubpath
	global pubway
	global pubpoints
	global pubexplore

	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = globalResolution 
	cells.cell_height = globalResolution

	for i in range(len(list)): 
		point=Point()
		#print list[i]
		point.x=list[i][0]
		point.y=list[i][1]
		if num == 1:
			point.z = 0
		if num == 2:
			point.z = .25
		if num == 3:
			point.z = 0.5
		if num == 4:
			point.z = 0.75
		if num == 5:
			point.z = .2
		cells.cells.append(point)
		#print point

	if num == 1:
		pub.publish(cells)
	if num == 2:
		pubpath.publish(cells)
	if num == 3:
		pubway.publish(cells) 
	if num == 4:
		pubpoints.publish(cells)
	if num == 5:
		pubexplore.publish(cells)


#Main handler of the project
def run():
    global xPosition
    global yPosition
    global localMap
    goal = [goalX, goalY]
    xPos = int((xPosition - 0.5*globalResolution - globalOffsetX)/globalResolution)
    yPos = int((yPosition + 0.5*globalResolution - globalOffsetY)/globalResolution)
    start = [xPos, yPos]
    print 'Starting Path Planning...'
    print 'Start: ', start
    print 'Goal: ', goal
    showCells([start, goal], 4)
    moveWithAStar(start, goal)
    


if __name__ == '__main__':
	global pose
	global pubmotion
	global pub
	global pubpath
	global pubway
	global pubpoints
	global pubexplore
	global startPosX
	global startPosY
	rospy.init_node('lab3')

	pose = Pose()
	pubmotion = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
	globalsub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, globalMapCallBack)
	localsub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, localMapCallBack)
	pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
	pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)  
	pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
	pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
	pubpoints = rospy.Publisher("/points", GridCells, queue_size=1)
	pubexplore = rospy.Publisher("/explore", GridCells, queue_size=1)
	goal_sub = rospy.Subscriber('/rbe/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
	localupdates = rospy.Subscriber('/move_base/local_costmap/costmap_updates', OccupancyGridUpdate, updateLocal, queue_size=1)
	globalupdates = rospy.Subscriber('/move_base/local_costmap/costmap_updates', OccupancyGridUpdate, updateGlobal, queue_size=1)


	rospy.Timer(rospy.Duration(.01), tCallback) # timer callback for robot location
	odom_list = tf.TransformListener() #listner for robot location
	rospy.sleep(.15)

	#Keeps the program going
	while not rospy.is_shutdown():
		rospy.spin