#!/usr/bin/env python
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
import rospy


class MyGlobals(object):
	robotPose = Pose()
	globalMap = OccupancyGrid()
	localMap = OccupancyGrid()
	mainMap = OccupancyGrid()
	obstacles = 70

	# Publishers
	pubExplored = rospy.Publisher("/explore", GridCells, queue_size=1)
	pubStart = rospy.Publisher("/start", GridCells, queue_size=1)
	pubEnd = rospy.Publisher("/end", GridCells, queue_size=1)
	pubTotalPath = rospy.Publisher("/total_path", GridCells, queue_size=1)
	pubWaypoints = rospy.Publisher("/waypoints", GridCells, queue_size=1)
	pubSubPath = rospy.Publisher("/sub_path", GridCells, queue_size=1)
	pubObstacles = rospy.Publisher("/obstacle", GridCells, queue_size=1)
	pubRobot = rospy.Publisher("/robot", PoseStamped, queue_size=1)
	pubMotion = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion