#!/usr/bin/env python

from settings import MyGlobals
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math
import rospy

# Publishes a twist message to drive the robot
# lin_Vel: Float type of linear velocity for robot
# ang_Vel: Float type of angular velocit for robot
def publishTwist(lin_Vel, ang_Vel):
	"""Send a movement (twist) message."""
	msg = Twist()
	msg.linear.x = lin_Vel
	msg.angular.z = ang_Vel
	MyGlobals.pubMotion.publish(msg)



# goal: Pose object to calculate orientation angle on
def angleFromPose(goal):
	quat = goal.orientation
	q = [quat.x, quat.y, quat.z, quat.w]
	roll, pitch, yaw = euler_from_quaternion(q)
	return yaw * (180/math.pi)



# angle: Float type of angle in degrees to rotate to
def rotate(angle):
	speed = 0.25
	vel = Twist();
	direction = 1

	print 'Rotating...'

	# set rotation direction
	error = angle - angleFromPose(MyGlobals.robotPose)
	if (error > -180 and error < 0):
		direction = -1

	# Turns until getting the error gets small
	while ((abs(error) >= 3) and not rospy.is_shutdown()):

		publishTwist(0, direction*speed)
		rospy.sleep(0.05)    
		error = angle - angleFromPose(MyGlobals.robotPose) 
	vel.angular.z = 0.0
	MyGlobals.pubMotion.publish(vel)

	rospy.sleep(2)



#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(distance):
	speed = 0.3 
	"""This function accepts a speed and a distance for the robot to move in a straight line"""

	print 'Driving...'

	initialX = MyGlobals.robotPose.position.x
	initialY = MyGlobals.robotPose.position.y
	atTarget = False

	#Loop until the distance between the attached frame and the origin is equal to the
	#distance specified 
	while (not atTarget and not rospy.is_shutdown()):
		currentX = MyGlobals.robotPose.position.x
		currentY = MyGlobals.robotPose.position.y
		currentDistance = math.sqrt((currentX-initialX)**2 + (currentY-initialY)**2) #Distance formula
		if (currentDistance >= distance):
			atTarget = True
			publishTwist(0, 0)

		else:
			publishTwist(speed, 0)
			rospy.sleep(0.05)




# goal: Pose object of goal position to move straightline distance to
def driveToPose(goal):

	tolerance = 0.1
	maxLinSpeed = 0.4
	maxAngSpeed = 0.4
	kp = 2
	distance = 1 # allows code to enter loop

	xGoal = goal.position.x
	yGoal = goal.position.y

	while distance >= 0.02:
		xPos = MyGlobals.robotPose.position.x
		yPos = MyGlobals.robotPose.position.y

		distance = (xGoal - xPos)**2 + (yGoal - yPos)**2

		print MyGlobals.robotPose 

		print 'Distance Away: ', distance

		linear = min(maxLinSpeed, distance*kp)

		print 'Linear Speed ', linear
		angular =  0
		publishTwist(linear, angular)

	publishTwist(0, 0)



# goal: Pose object of goal position to go to
# orientMatter: Boolean type stating if the final orientation matters
def navToPose(goal, orientMatter):

	print 'Navigating to Position...'
	print goal
	print MyGlobals.robotPose

	# robots position
	xPos = MyGlobals.robotPose.position.x
	yPos = MyGlobals.robotPose.position.y

	# goal position
	xGoal = goal.position.x
	yGoal = goal.position.y

	# Rotate to correct orientation for straightline movement to goal
	angle = math.degrees(math.atan2(yGoal - yPos, xGoal - xPos))

	print 'Rotate to: ', angle
	rotate(angle)


	#driveToPose(goal)

	distance = math.sqrt((xGoal - xPos)**2 + (yGoal - yPos)**2)

	print 'Drive: ', distance

	driveStraight(distance)

	# rotates to goal orientation when final orientation matters
	if orientMatter:
		print 'Spinning to Final Orientation...'
		endAngle = angleFromPose(goal)
		rotate(endAngle)

	print 'Done Navigating'