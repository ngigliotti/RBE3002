#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
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
    localMap = data
    print 'Local Map Updated'


# reads in global map
def globalMapCallBack(data):
	global globalMap
	globalMap = data
	print 'Global Map Updated'


#keeps track of current location and orientation
def tCallback(event):
    global pose
    global xPosition
    global yPosition
    global theta

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
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
	global resolution
	global mapData
	global width
	global height
	global offsetX
	global offsetY

	# Intializes on the global Map
	data = globalMap
	mapgrid = data
	resolution = data.info.resolution
	mapData = data.data
	width = data.info.width
	height = data.info.height
	offsetX = data.info.origin.position.x
	offsetY = data.info.origin.position.y

	goalX = goal.pose.position.x
	goalY = goal.pose.position.y
	quat = goal.pose.orientation
	q = [quat.x, quat.y, quat.z, quat.w]
	roll, pitch, yaw = euler_from_quaternion(q)
	desiredT = yaw * (180/math.pi)

	goalX = int((goalX - 0.5*resolution - offsetX)/resolution)
	goalY = int((goalY + 0.5*resolution - offsetY)/resolution)

	run()


def navToPose(goalX, goalY):
	global xPosition
	global yPosition
	global theta

	print 'Navigating to pose...'

	#capture desired x and y positions
	desiredY = goalY
	desiredX = goalX

    #compute angle required to make straight-line move to desired pose
	angle = math.degrees(math.atan2(desiredY - yPosition, desiredX - xPosition))

    #compute distance to target
	distance = math.sqrt((desiredX - xPosition)**2 + (desiredY - yPosition)**2)	

	print 'Spinning to Angle: %d' % angle #turn to calculated angle
	rotate(0.3, angle)

	print 'Moving Distance: %d' % distance #move in straight line specified distance to new pose
	driveStraight(0.1, distance)


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



def aStar(start,goal, data):
	global cost
	global path
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

	# Estimates the distance to the goal
	def h(point, goal):
		# Manhattan distance with diagonal movement
		#return max(abs(goal[0] - point[0]), abs(goal[1] - point[1]))
		return math.sqrt((goal[0] - point[0])**2 + (goal[1]- point[1])**2)

	def isWall(point):
		value = mapData[point[1] * (width) + point[0]]
		if value >= 65:
			return True	
		return False		#Else
		print "Found A Wall"

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
			showCells(path, 2)		# Displays Path
			showCells([],5)
			rospy.sleep(1)
			if cost > 1:
				showCells(wayPoints(path), 3)		#Displays Waypoints
			break

		if node not in visited:
			visited.append(node)

		if isWall(node):
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

			if (currentAngle != lastAngle): #if the angle has changed
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


def moveWithAStar(start, goal):
	global xPosition
	global yPosition
	global path
	global resolution
	global offsetX
	global offsetY
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
	aStar(start, goal, globalMap)
	way = wayPoints(path)
	showCells(path, 2)
	showCells(way, 3)

	rospy.sleep(5)

	while len(way) > 1 and len(path) > 2:
		# Calculates A* on the Local map from start to first waypoint
		way1 = way[0]
		print way
		print way1
		print 'Calculating AStar on Local Map...'
		showCells([start, way1], 4)
		aStar(start, way1, localMap) # Calculates A* on the local map from start to first waypoint
		tempWay = wayPoints(path)
		showCells(path, 2)
		showCells([start, tempWay[0]], 4)

		wayX = ((tempWay[0][0] + xPosition)*resolution)+offsetX + (.5 * resolution)
		wayY = ((tempWay[0][1] + yPosition)*resolution)+offsetY - (.5 * resolution)
		print [wayX, wayY]
		navToPose(wayX, wayY)

		# Updates Starting Position
		xPosition = int((xPosition - 0.5*resolution - offsetX)/resolution)
		yPosition = int((yPosition + 0.5*resolution - offsetY)/resolution)

		# Calculates A* on the Global map from current waypoint to goal
		start = [xPosition, yPosition]
		print 'Calculating AStar on Global Map...'
		aStar(start, goal, globalMap)
		way = wayPoints(path)
		showCells(path, 2)
		showCells(way, 3)


	goalX = (goal[0]*resolution)+offsetX + (.5 * resolution)
	goalY = (goal[1]*resolution)+offsetY - (.5 * resolution)

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

	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution 
	cells.cell_height = resolution

	for i in range(len(list)): 
		point=Point()
		#print list[i]
		point.x=(list[i][0]*resolution)+offsetX + (.5 * resolution) # added secondary offset 
		point.y=(list[i][1]*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
		point.z=0
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
    goal = [goalX, goalY]
    xPosition = int((xPosition - 0.5*resolution - offsetX)/resolution)
    yPosition = int((yPosition + 0.5*resolution - offsetY)/resolution)
    start = [xPosition, yPosition]
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
	localsub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, localMapCallBack)
	pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
	pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)  
	pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
	pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
	pubpoints = rospy.Publisher("/points", GridCells, queue_size=1)
	pubexplore = rospy.Publisher("/explore", GridCells, queue_size=1)
	goal_sub = rospy.Subscriber('/rbe/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results

	rospy.Timer(rospy.Duration(.01), tCallback) # timer callback for robot location
	odom_list = tf.TransformListener() #listner for robot location
	rospy.sleep(.15)

	#Keeps the program going
	while not rospy.is_shutdown():
		rospy.spin()
