#!/usr/bin/env python

from settings import MyGlobals
from geometry_msgs.msg import Twist
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
	quat = goal.pose.orientation
	q = [quat.x, quat.y, quat.z, quat.w]
	roll, pitch, yaw = euler_from_quaternion(q)
	return yaw * (180/math.pi)



# angle: Float type of angle in degrees to rotate to
def rotate(angle):

	kp = .01
	maxSpeed = 0.5 
	tolerance = 1
	error = 1 # sets original error value so it originally enters loop

    # transforms the input angle into range 0 to 360 deg
	angle = angle % 360
	if angle < 0:
		angle += 360

	print 'Rotating...'
	print angle
	# Rotates the robot until it gets close to the correct angle
	while (abs(error) >= tolerance and not rospy.is_shutdown()):
		# Calculates the current robots angle in range o to 360 deg
		robotAngle = math.degrees(MyGlobals.robotPose.orientation.z)
		if robotAngle < 0:
			robotAngle += 360

		error = angle - robotAngle

		angular = max(maxSpeed, min(-maxSpeed, kp*error))

		print 'Speed: ', angular
		publishTwist(0, angular)

	# Stops the robot once it is close to the correct angle
	print 'Done Rotating'  
	publishTwist(0, 0)



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

		linear = max(maxLinSpeed, distance*kp)
		angular =  0
		publishTwist(linear, angular)

	publishTwist(0, 0)



# goal: Pose object of goal position to go to
# orientMatter: Boolean type stating if the final orientation matters
def navToPose(goal, orientMatter):

	print 'Navigating to Position...'
	print goal

	# robots position
	xPos = MyGlobals.robotPose.position.x
	yPos = MyGlobals.robotPose.position.x

	# goal position
	xGoal = goal.position.x
	yGoal = goal.position.y

	# Rotate to correct orientation for straightline movement to goal
	angle = math.degrees(math.atan2(yGoal - yPos, xGoal - xPos))
	rotate(angle)

	# Drives the straighline path to the goal
	driveToPose(goal)

	# rotates to goal orientation when final orientation matters
	if orientMatter:
		print 'Spinning to Final Orientation...'
		endAngle = angleFromPose(goal)
		rotate(endAngle)

	print 'Done Navigating'