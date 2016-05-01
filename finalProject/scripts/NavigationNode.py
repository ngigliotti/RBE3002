#!/usr/bin/env python

# Imports 
import rospy
rospy.init_node('finalProject')
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
from display import expandObstacles, clearAllDisplay
import drive
from pathPlanning import pathPlanningNav
from settings import MyGlobals
from roomMapping import mapRoom



# Reads in the Local Map
def localMapCallback(data):
	MyGlobals.localMap = data



# Reads in the Global Map
def globalMapCallback(data):
	MyGlobals.globalMap = data
	expandObstacles(MyGlobals.globalMap, MyGlobals.obstacles)



def mapCallback(data):
	MyGlobals.mainMap = data



# Updates the Local Map
def updateLocal(data):
	updateMap(data, MyGlobals.localMap)



# Updates the Global Map
def updateGlobal(data):
	updateMap(data, MyGlobals.globalMap)
	expandObstacles(MyGlobals.globalMap, MyGlobals.obstacles)



# data: OccupancyGridUpdate object containing update to map
# map: OccupancyGrid object to apply update to
def updateMap(data, map):
	update = data.data
	w_u = data.width
	h_u = data.height
	x_u = data.x
	y_u = data.y

	mapData = list(map.data)
	w_m = map.info.width

	index_u = 0

	index = x_u + w_m*(y_u - 1)
	for i in range(h_u):
		for j in range(w_u):
			mapData[index] = update[index_u]
			index_u += 1
			index += 1
		index += w_m - w_u



# Keeps the position of the robot updated
def positionCallback(event):

	# Transforms to the map frame from the base frame
	transform.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
	(pos, orient) = transform.lookupTransform('map', 'base_footprint', rospy.Time(0))

	# Updates the x and y values for the robot pose
	MyGlobals.robotPose.position.x = pos[0]
	MyGlobals.robotPose.position.y = pos[1]

	# Updates the orientation for the robot pose
	q = [orient[0], orient[1], orient[2], orient[3]]
	roll, pitch, yaw = euler_from_quaternion(q)
	MyGlobals.robotPose.orientation.x = orient[0]
	MyGlobals.robotPose.orientation.y = orient[1]
	MyGlobals.robotPose.orientation.z = orient[2]
	MyGlobals.robotPose.orientation.w = orient[3]


	# Publishes the robots position for visual use in rviz
	publishPose = PoseStamped()
	publishPose.pose = MyGlobals.robotPose
	publishPose.header.frame_id = 'map'
	publishPose.header.stamp = rospy.Time(0)
	MyGlobals.pubRobot.publish(publishPose)



# Reads the Goal Position After Selecting RViz location
def readGoal(goal):
	clearAllDisplay()
	mapRoom()



if __name__ == '__main__':

	

	# Subscribers
	globalCostmap = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, globalMapCallback)
	localCostmap = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, localMapCallback)
	localupdates = rospy.Subscriber('/move_base/local_costmap/costmap_updates', OccupancyGridUpdate, updateLocal, queue_size=1)
	globalupdates = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, updateGlobal, queue_size=1)
	subGoal = rospy.Subscriber('/rbe/goal', PoseStamped, readGoal, queue_size=1)
	mainMap = rospy.Subscriber('/map', OccupancyGrid, mapCallback)

	# timer callback for robot location
	rospy.Timer(rospy.Duration(.01), positionCallback) 

	# transform listener
	transform = tf.TransformListener() 

	rospy.sleep(.1)

	#Keeps the program going
	while not rospy.is_shutdown():
		rospy.spin