#!/usr/bin/python
import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import math
from math import atan2
from sensor_msgs.msg import LaserScan
import numpy as np

class Robot:
	#Class variables
	rospy.init_node("husky")
	#Constructor
	def __init__(self, queueSize, frequency, goal):
		self.ARRAY_SIZE = 720
		self.FRONT_MIN = 240 #240
		self.FRONT_MAX = 479
		self.currentPosition = Point()
		self.goal = goal
		self.speed = Twist()
		self.yaw = 0.0
		self.frequency = frequency
		self.rate = rospy.Rate(self.frequency)
		self.queueSize = queueSize
		self.velocityPub = rospy.Publisher("/cmd_vel", Twist, queue_size = self.queueSize)
		self.positionSub = rospy.Subscriber("/current_position", Point, self.position)
		self.lidarSub = rospy.Subscriber("/scan", LaserScan, self.lidarData)
		self.inc_x = self.goal.x - self.currentPosition.x
		self.inc_y = self.goal.y - self.currentPosition.y
		self.lidarValues = LaserScan()
		self.MIN_DISTANCE = 20
		self.MIN_DETECTION = 8
		self.MIN_GAP_LENGTH = 3
		self.MIN_GAP_WIDTH = 1.5
		self.ANGLE_INCREMENT = 0.0065540750511 #Angle between lasers 
		self.MAX_DISTANCE = 30
		
		
		
		
		

	#Callback function
	def lidarData(self, data):
		self.lidarValues = data


	def position(self, position):
		self.currentPosition.x = position.x
		self.currentPosition.y = position.y
		self.yaw = position.z
		self.updateIncrements()

	#Instance methods

	#Lidar combined with move methods
	
	#def detectGap(self):
	#	if len(self.lidarValues.ranges) == 720:
	#		for index in range(self.FRONT_MIN, self.FRONT_MAX+1):
	#			if not math.isinf(self.lidarValues.ranges[index]):
	#				self.moveToCenterOfGap()
	#			else:
	#				self.moveToPosition()
	
	#def obstacleAvoidance(self):
		#while self.detectCollision():
			#Store current position and calc distance to center of gap
		#	self.moveToCenterOfGap()
		#self.moveToPosition()
			

	#def getLidarTurnDirection(self):
	#	if len(self.getMaxGap()) != 0:
	#		return (((math.degrees(self.getLidarAngleToGoal()) - math.degrees(self.getYaw()) + 540) % 360) - 180)

	#def getLidarAngleToGoal(self):
		#rospy.loginfo(self.getMaxGap())
	#	if len(self.getMaxGap()) != 0:
	#		return self.getMaxGap()[2]
		

	#def getLidarDifferenceBetweenAngles(self):
	#	if len(self.getMaxGap()) != 0:
			#rospy.loginfo(abs(self.getLidarAngleToGoal() - self.getYaw()))
	#		return abs(self.getLidarAngleToGoal() - self.getYaw())
	
		
	#def getStraightLinePath(self):
	#def distanceToCenterOfGap(self):
	#	maxGap = self.getMaxGap()
		#Split angle in half 
		#Use either left or right side
		#Find the corner angle using the original left, right, and middle angle of the whole triangle
		#Use half the gap distance to find the right angle toward the center gap
		
		#rospy.loginfo(maxGap)
	#	if len(maxGap) != 0:
	#		middleAngle = maxGap[2]/2
			#leftIndex, rightIndex, middleAngle, gapDistance
	#		a = self.lidarValues.ranges[maxGap[1]-1]
	#		b = self.lidarValues.ranges[maxGap[0]]
	#		rospy.loginfo("Right val: " +str(a))
	#		rospy.loginfo("Left val: " + str(b))
	#		rospy.loginfo("Middle angle: " + str(maxGap[2])) 
	#		rospy.loginfo("Gap distance: " + str(maxGap[3]))
	#		c =  maxGap[3]/2
			
	#		if math.isinf(a) and math.isinf(b):
	#			if maxGap[1] == 480:
	#				b = self.lidarValues.ranges[maxGap[0]-1]
	#				a = b
				
	#			elif maxGap[0] == 240:
	#				a = self.lidarValues.ranges[maxGap[1]]
	#				b = a
				
	#			else:
	#				a = self.lidarValues.ranges[maxGap[1]]
	#				b = self.lidarValues.ranges[maxGap[0]-1]
	#		rospy.loginfo("New right val: " +str(a))
	#		rospy.loginfo("New left val " + str(b))
	#		rospy.loginfo("Half mid angle: " + str(maxGap[2]/2))
	#		rospy.loginfo("Half gap distance: " +str(c))

			#Sin law
	#		sinC = (math.sin(middleAngle)/c)
	#		betaAngle = np.arcsin((b*sinC))

			#Inside angles of triangle add to 180
	#		alphaAngle = 180 - middleAngle - betaAngle

			#Equation for distance to center is a = csin(alpha)/sin(gamma)		
	#		sinAlpha = math.sin(alphaAngle)
	#		csinAlpha = c * sinAlpha
	#		sinGamma = math.sin(middleAngle)
	#		distanceToCenter = csinAlpha/sinGamma

	#		return (distanceToCenter, middleAngle)# Distance to center is nan for some reason.
	#	else:
	#		return (0, 0)

	def distanceToCenterOfGap(self):
		#(leftIndex, rightIndex, middleAngle, gapDistance)
		maxGap = self.getMaxGap()
		#rospy.loginfo("a in distance: " + str(self.lidarValues.ranges[maxGap[1]]))
		#rospy.loginfo("b in distance: " + str(self.lidarValues.ranges[maxGap[0]]))
		#Split c in half
		#Use either left or right side
		#Find the corner angle using the original left, right, and middle angle of the whole triangle
		#Half the gamma angle, can use that to find the x and y coordinates using sohcahtoa
		#cos and sin law to find the alpha angle and distance to center
		#Add the x and y increments to currentposition to create a new goal point at the center of the gap
		#Find the angle using original functions and turn toward center of gap
		#Move toward center of gap
		#May have to split angle in half because triangle distance won't start from the Husky current position

		#No longer need to adjust the rightIndex value
		#Using the c/2 method to find the distance to center instead of the gamma/2 (middleAngle/2) method
		if len(maxGap) != 0:
			#Have to pass in the new a or b value to newGoalPoint function becaus
			#Right side of the gap
			rightIndex = maxGap[1]
			a = self.lidarValues.ranges[rightIndex]

			#Left side of the gap
			leftIndex = maxGap[0]
			b = self.lidarValues.ranges[leftIndex]

			#Adjust if the left or right side ends on infinity
			if math.isinf(a):
				a = b
			if math.isinf(b):
				b = a
			rospy.loginfo("Right index: " + str(a) + ", " + str(rightIndex))
			rospy.loginfo("Left index: " + str(b) + ", " + str(leftIndex))

			#Distance between left and right side
			c = maxGap[3]
	
			#Angle between a and b
			gamma = maxGap[2]
			

			#Finding alpha angle, top left angle of the triangle (Angle between b and c)
			#sin laws: sin(alpha)/a = sin(gamma)/c
			#alpha = arcsin(a(sin(gamma)/c))
			sinGamma = math.sin(gamma)/c
			aSinGamma = a*sinGamma
			alpha = np.arcsin(aSinGamma)
			beta = 180 - math.degrees(gamma+alpha)
			#rospy.loginfo("Alpha: " + str(math.degrees(alpha)))

			halfc = c/2
			
			#Cosine law: a = sqrt(b^2 + halfc^2 - 2*b*halfc*cos(alpha))

			cosine = 2 * b * halfc * math.cos(alpha)
			centerDistance = math.sqrt(b**2 + halfc**2 - cosine)

			
			#New gamma angle

			
			sinAlpa = math.sin(alpha)/centerDistance

			leftGamma = np.arcsin(halfc * sinAlpa)
			rightGamma = gamma - leftGamma

			#rospy.loginfo("Left index: " + str(maxGap[0]))
			#rospy.loginfo("Left index val: " + str(b))
			#rospy.loginfo("Right index: " + str(maxGap[1]))
			#rospy.loginfo("Right index val: " + str(a))
			#rospy.loginfo("Gap distance: " + str(c))
			#rospy.loginfo("Gamma: " + str(math.degrees(gamma)))
			#rospy.loginfo("Distance to center: " + str(centerDistance))
			#rospy.loginfo("Left gamma: " + str(math.degrees(leftGamma)))
			#rospy.loginfo("Right gamma: " + str(math.degrees(rightGamma)))
			#rospy.loginfo("Gamma: " + str(gamma))
			return (maxGap[0], maxGap[1], leftGamma, rightGamma, centerDistance)
		
	def newGoalPoint(self):
		values = self.distanceToCenterOfGap()
		newGoal = Point()
		if values != None:
			leftIndex = values[0]
			rightIndex = values[1]
			leftGamma = values[2]
			rightGamma = values[3]
			distanceToCenter = values[4]
		#If the left and right index are less than 360, then use rightgamma, else use left gamma for the angle toward goal
		#Add the remaining angle toward the goal using the difference from index 360 and either the left index if both indexes are greater than 360 or the right index if the both index are less than 360
		
		#Use SOHCAHTOA to find the x and y difference to add to current position and find new goal point
			if leftGamma > rightGamma:
				angleToGoal = leftGamma
			else:
				angleToGoal = rightGamma	
			rospy.loginfo("Angle to goal: " + str(angleToGoal))			
			if leftIndex > 360 and rightIndex > 360:
				yDifference = distanceToCenter * math.sin(leftGamma)
				xDifference = distanceToCenter * math.cos(leftGamma)
				#rospy.loginfo("x value: " + str(xDifference))
				#rospy.loginfo("y value: " + str(yDifference))
				#rospy.loginfo("Distance to center: " + str(distanceToCenter))
				rospy.loginfo("Left gamma: " + str(leftGamma))
				rospy.loginfo("Right gamma: " + str(rightGamma))
				#rospy.loginfo("Left plus right: " + str(leftGamma+rightGamma))
				#rospy.loginfo(self.getCurrentPosition())
				newGoal.x = self.getCurrentPosition().x + xDifference
				newGoal.y = self.getCurrentPosition().y - yDifference
				#rospy.loginfo(newGoal)
			elif leftIndex < 360 and rightIndex < 360:
				yDifference = distanceToCenter * math.sin(rightGamma)
				xDifference = distanceToCenter * math.cos(rightGamma)
				#rospy.loginfo("x value: " + str(xDifference))
				#rospy.loginfo("y value: " + str(yDifference))
				#rospy.loginfo("Distance to center: " + str(distanceToCenter))
				#rospy.loginfo(self.getCurrentPosition())
				newGoal.x = self.getCurrentPosition().x + xDifference
				newGoal.y = self.getCurrentPosition().y + yDifference
				#rospy.loginfo(newGoal)
			
		return newGoal

	def getAngleTowardCenter(self):
		goalPoint = self.newGoalPoint()
		return atan2(goalPoint.y, goalPoint.x)	

	def getLidarTurnDirection(self):
		return (((math.degrees(self.getAngleTowardCenter()) - math.degrees(self.getYaw()) + 540) % 360) -180)

	def getLidarDifferenceBetweenAngles(self):
		return abs(self.getAngleTowardCenter() - self.getYaw())

	def detectObstacle(self):
		self.rate.sleep()
		if len(self.lidarValues.ranges) != 0:
			for vals in range(345, 376):
				if self.lidarValues.ranges[vals] < self.MIN_DETECTION:
					return True
			return False
			
				 
	#def distanceToCenter(self):

	def alignWithGoal(self):

		speed = Twist()
		while self.getDifferenceBetweenAngles() > 0.1:
			if self.getTurnDirection() > 0:
				speed.angular.z = 0.5
				speed.linear.x = 0.0
				rospy.loginfo("Turning left")
			else:
				speed.angular.z = -0.5
				speed.linear.x = 0.0
				rospy.loginfo("Turning right")
			self.setSpeed(speed)
			self.publishVelocity()
	def moveToGoal(self):
		
		while not self.goalReached():
			self.moveToPosition()
			rospy.loginfo("Distance remaining to goal: " + str(self.getDistanceRemaining()))
			if self.detectObstacle():
				newGoal = self.newGoalPoint()
				rospy.loginfo(newGoal)
				originalGoal = self.getGoal()
				rospy.loginfo("Obstacle detected")
				
				
				self.setGoal(newGoal)
				while not self.goalReached():
					rospy.loginfo("New goal: " + str(self.getGoal()))
					rospy.loginfo("Distance remaining to center of gap: " + str(self.getDistanceRemaining()))
					self.moveToPosition()
				self.setGoal(originalGoal)
				self.alignWithGoal()

		rospy.loginfo("Goal reached")
	
 
	def moveToCenterOfGap(self):
		#if not self.detectObstacle():
		#	self.moveToPosition()
		#else:
		#	newGoal = self.newGoalPoint()
		#	originalGoal = self.getGoal()
		#	self.setGoal(newGoal)
		#	while self.getDistanceRemaining() < 0.9:
		#		rospy.loginfo(self.getDistanceReamining())
		#		self.moveToPosition()
		#		rospy.loginfo("WhileLoop")
		#	self.setGoal(originalGoal)
		newGoal = self.newGoalPoint()
		if newGoal.x == 0 and newGoal.y == 0:
			rospy.loginfo("Goal is 0,0")
			
		else:
			originalGoal = self.getGoal()
			self.setGoal(newGoal)
			rospy.loginfo("New goal: " + str(newGoal))
			while self.getDistanceRemaining() > 0.09:
				rospy.loginfo("Distance remaining: " + str(self.getDistanceRemaining()))
				self.moveToPosition()
			self.setGoal(originalGoal)
			rospy.loginfo("Middle of gap reached")
		rospy.loginfo("Out of loop")
		rospy.loginfo(newGoal)
		#rospy.loginfo(newGoal)
		#self.moveToPosition()
		self.rate.sleep()
			
		
		

	#def getLidarTurnDirection(self):
	#	if len(self.getMaxGap()) != 0:
	#		return (((math.degrees(self.getLidarAngleToGoal()) - math.degrees(self.getYaw()) + 540) % 360) - 180)

	#def getLidarAngleToGoal(self):
	#	if len(self.getMaxGap()) != 0:
	#		return self.getMaxGap()[2]

	#def getLidarDifferenceBetweenAngles(self):
	#	if len(self.getMaxGap()) != 0:
	#		return abs(self.getLidarAngleToGoal() - self.getYaw())

	
	#Lidar methods

	#def centerGapPosition(self):
	#def calculateDistanceToCenter(self):
		#Center angle
		#Sides of gap
		#c value
		
	
		
		#Husky won't be able to pass through if the gap is less than 2 meters
	def calcGapWidth(self, leftIndex, rightIndex, middleAngle):
		#rospy.loginfo("Left index: " + str(leftIndex))
		#rospy.loginfo("Lef val: " + str(self.lidarValues.ranges[leftIndex]))
		#rospy.loginfo("Right index: " + str(rightIndex-1))
		#rospy.loginfo("Right val: " + str(self.lidarValues.ranges[rightIndex-1]))
		#rospy.loginfo("Middle angle: " + str(middleAngle))
		#right index ends on a value that isn't >= self.MIN_DISTANCE so we need to pass in rightIndex-1
		a = self.lidarValues.ranges[rightIndex-1]
		b = self.lidarValues.ranges[leftIndex]
		#rospy.loginfo("a: " + str(a))
		#rospy.loginfo("b: " + str(b))
		newMidAngle = middleAngle
		newLeftIndex = leftIndex
		newRightIndex = rightIndex-1
		#rospy.loginfo("a: " + str(a))
		#rospy.loginfo("b: " + str(b))
		#if math.isinf(a):
		#	a = self.MAX_DISTANCE
		#if math.isinf(b):
		#	b = self.MAX_DISTANCE

		#Different infinity situations, have to account for new mid angle
		if math.isinf(a) and math.isinf(b):
			if rightIndex == 480:
				b = self.lidarValues.ranges[leftIndex-1]
				a = b
				newMidAngle = middleAngle + self.ANGLE_INCREMENT
				newLeftIndex = leftIndex-1
				#rospy.loginfo("480")
			elif leftIndex == 240:
				a = self.lidarValues.ranges[rightIndex]
				b = a
				newRightIndex = rightIndex
				newMidAngle = middleAngle + self.ANGLE_INCREMENT
				#rospy.loginfo("240")
				
			else:
				newLeftIndex = leftIndex - 1
				newRightIndex = rightIndex
				a = self.lidarValues.ranges[newRightIndex]
				b = self.lidarValues.ranges[newLeftIndex]
				newMidAngle = middleAngle + (self.ANGLE_INCREMENT*2)
		#Have to pass in the new a or b value to newGoalPoint function because it uses the left side.
		#if math.isinf(b) and not math.isinf(a):
			
				
				#rospy.loginfo("Old right index: " + str(newRightIndex))
				
				#rospy.loginfo("middle")
				#rospy.loginfo("a: " + str(a) + ", " + str(newRightIndex)+ ", " + str(self.lidarValues.ranges[newRightIndex]))
				#rospy.loginfo("b: " + str(b) + ", " + str(newLeftIndex) + ", " + str(self.lidarValues.ranges[newLeftIndex]))
				#rospy.loginfo("Old mid angle: " + str(middleAngle))
				#rospy.loginfo("midAngle: " + str(newMidAngle))
		
		#rospy.loginfo("a in calcwidth: " + str(a))
		#rospy.loginfo("b in calc width: " + str(b))
		cosine = (2 * a * b * math.cos(newMidAngle))
		#rospy.loginfo("Old mid angle: " +str(middleAngle))
		#rospy.loginfo("New mid angle: " + str(newMidAngle))
		c = math.sqrt(a**2 + b**2 - cosine)
		#rospy.loginfo("Distance: " + str(c))
		self.rate.sleep() #Remove once finished
		#rospy.loginfo("New mid angle: " + str(newMidAngle))
		
		return (c, newMidAngle, newLeftIndex, newRightIndex)

	
	def getMaxGap(self):
		
		leftIndex = self.FRONT_MIN
		rightIndex = self.FRONT_MIN
		gapSize = 0
		gapDistance = -1
		maxGap = ()
		#Check if the array is filled
		#testArray = [inf, inf, inf, inf, 8.0, 5.0, 6.0, 5.0, 5.0, inf, inf, inf]
		if len(self.lidarValues.ranges) != 0:
			while (rightIndex < self.FRONT_MAX+1):
				if self.lidarValues.ranges[rightIndex] >= self.MIN_DISTANCE or math.isinf(self.lidarValues.ranges[rightIndex]):
					if leftIndex != rightIndex and gapSize == 0:
						leftIndex = rightIndex

					gapSize += 1
					rightIndex += 1
					
					#Detects the gap value at the end of the array
					if rightIndex == (self.FRONT_MAX+1):
						if gapSize >= self.MIN_GAP_LENGTH:
							#Gap size is one less because 5 array values only have 4 gaps between them
							middleAngle = (gapSize-1) * self.ANGLE_INCREMENT
							#rospy.loginfo("Previous midangle: " + str(middleAngle))
							#self.calcGapWidth(leftIndex, rightIndex, middleAngle)
							gapWidth = self.calcGapWidth(leftIndex, rightIndex, middleAngle)
							#gapWidth = temp[0]
							#middleAngle = temp[1]
							if gapWidth[0] >= self.MIN_GAP_WIDTH:
								if gapWidth[0] > gapDistance:
									gapDistance = gapWidth[0]
									maxGap = (gapWidth[2], gapWidth[3], gapWidth[1], gapDistance)
									#rospy.loginfo("Old left index: " + str(leftIndex))
									#rospy.loginfo("Old right index: " + str(rightIndex))
									#rospy.loginfo("Left index val: " + str(self.lidarValues.ranges[gapWidth[2]]))
									#rospy.loginfo("Right index val: " + str(self.lidarValues.ranges[gapWidth[3]]))
									#rospy.loginfo("Gap size: " + str(gapSize))
									#rospy.loginfo(gapWidth[1])
				
				else:
					#Passes in a value for rightIndex that is not the minimum distance
					if gapSize >= self.MIN_GAP_LENGTH:
						middleAngle = (gapSize-1) * self.ANGLE_INCREMENT
						#rospy.loginfo("Previous midangle: " + str(middleAngle))
						#self.calcGapWidth(leftIndex, rightIndex, middleAngle)
						gapWidth = self.calcGapWidth(leftIndex, rightIndex, middleAngle)
						#gapWidth = temp[0]
						#middleAngle = temp[1]
						if gapWidth[0] >= self.MIN_GAP_WIDTH:
							if gapWidth[0] > gapDistance:
								gapDistance = gapWidth[0]
								maxGap = (gapWidth[2], gapWidth[3], gapWidth[1], gapDistance)
								#rospy.loginfo("Old left index: " + str(leftIndex))
								#rospy.loginfo("Old right index: " + str(rightIndex))
								#rospy.loginfo("Left index val: " + str(self.lidarValues.ranges[gapWidth[2]]))
								#rospy.loginfo("Right index val: " + str(self.lidarValues.ranges[gapWidth[3]]))
								#rospy.loginfo("Gap size: " + str(gapSize))
								#rospy.loginfo(maxGap)
					
					gapSize = 0
					rightIndex += 1
		#Check if tuple is empty and determine what husky will do if no gap is detected
		#Returns tuple (leftIndex, rightIndex, middleAngle, gapDistance)
		#rospy.loginfo(maxGap)
		#rospy.loginfo(self.getYaw())
		self.rate.sleep() #Remove once finished
		#if len(maxGap) != 0:
			#rospy.loginfo("a in maxgap: " +  str(self.lidarValues.ranges[maxGap[1]]))
			#rospy.loginfo("b in maxgap: " + str(self.lidarValues.ranges[maxGap[0]]))
		return maxGap


	#Get gap with test array
	#def getGap(self):
	#	testArray = [5.0, 5.1, 6.0, 7.0, 8.0, 3.0, 6.0, 5.0, 6.0, 8.0, 4.0, 5.0]
	#	leftIndex = 0
	#	rightIndex = 0
	#	gapSize = 0
	#	maxGap = -1
		#[0.5, 5.1, 6.0 , 7.0. inf, 3.0, inf, 3.0, inf, 8.0, 1.0, 3.0]
		#if len(self.lidarValues.ranges) == self.ARRAY_SIZE:
	#	while rightIndex < len(testArray):
			#Check if the value is greater than or equal to the min distance
	#		if testArray[rightIndex] >= self.MIN_DISTANCE:
				
	#			if leftIndex != rightIndex and gapSize == 0:
	#				leftIndex = rightIndex
	#			gapSize += 1
	#			rightIndex += 1

				#Detects the gap at the end of the array
	#			if rightIndex == len(testArray):
	#				if gapSize >= self.MIN_GAP_LENGTH:
	#					self.storeGaps(leftIndex, rightIndex, gapSize)
					
	#		else:
				#Right index ends on a value that is not the minimum distance
	#			if gapSize >= self.MIN_GAP_LENGTH:
	#				self.storeGaps(leftIndex, rightIndex, gapSize)
	#			gapSize = 0
	#			rightIndex += 1
					
		
		
		
	
		
	

	#def detectGap(self):
	#	index = self.FRONTMIN
	#	indexDic = {}
	#	if len(self.lidarValues.ranges)==720:		
			#Gap is 3 consecutive values with a distance more than 5 meters
	#		for val in range(self.FRONTMIN, self.FRONTMAX+1):
	#			if self.lidarValues.ranges[val] >=1:
	#				indexDic[val] = self.lidarValues.ranges[val]
	#		self.searchGaps(indexDic)
			
	#Husky methods			
	def goalReached(self):
		if self.getDistanceRemaining() < 0.09:
			rospy.loginfo("Goal reached")
			return True
		return False

	#Updates x and y difference between Husky and goal
	def updateIncrements(self):
		self.inc_x = self.goal.x - self.currentPosition.x
		self.inc_y = self.goal.y - self.currentPosition.y

	#Angle between Husky and goal
	def getAngleToGoal(self):
		return atan2(self.inc_y, self.inc_x)

	#Difference between angle to goal and direction Husky is facing
	def getDifferenceBetweenAngles(self):
		return abs(self.getAngleToGoal() - self.getYaw())

	#Turn left if value is positive, right if negative
	def getTurnDirection(self):
		return (((math.degrees(self.getAngleToGoal()) - math.degrees(self.getYaw()) + 540) % 360) - 180)

	#Distance to goal
	def getDistanceRemaining(self):
		return math.sqrt((self.inc_x**2) + (self.inc_y**2))

	#Position robot will move too
	def setGoal(self, newGoal):
		self.goal = newGoal	

	#Returns current position of Husky, x, y, and yaw
	def getCurrentPosition(self):
		return self.currentPosition

	#Returns yaw
	def getYaw(self):
		return self.yaw

	#Return goal position
	def getGoal(self):
		return self.goal

	#Set robots speed using Twist()	
	def setSpeed(self, speed):
		self.speed = speed

	#Return speed object
	def getSpeed(self):
		return self.speed

	#Needs to be tested, sets a new queue size
	def setQueueSize(self, newSize):
		self.queueSize = newSize
		self.velocityPub = self.velocityPub = rospy.Publisher("/cmd_vel", Twist, queue_size = self.queueSize)

	#New publishing frequency
	def setFrequency(self, frequency):
		self.frequency = frequency
		self.rate = rospy.Rate(self.frequency)

	#Return current queue size
	def getQueueSize(self):
		return self.queueSize

	#Return current publishing frequency
	def getFrequency(self):
		return self.frequency

	#Publish speed (Twist()) to velocity topic
	def publishVelocity(self):
		self.velocityPub.publish(self.speed)
		self.rate.sleep()

	#Move to goal point
	def moveToPosition(self):
		#rospy.loginfo("Distance remaining: " + str(self.getDistanceRemaining()))
		speed = Twist()
		if self.getDifferenceBetweenAngles() > 0.1:
			if self.getTurnDirection() > 0:
				speed.angular.z = 0.5
				speed.linear.x = 0.0
				rospy.loginfo("Turning left")
			else:
				speed.angular.z = -0.5
				speed.linear.x = 0.0
				rospy.loginfo("Turning right")
		else:
			speed.linear.x = 0.5
			speed.angular.z = 0.0
			rospy.loginfo("Aligned")
		self.setSpeed(speed)
		self.publishVelocity()
		
		

goal = Point()
newGoal = Point()
#goal.x = 5.5744154127

#goal.y = -0.876141815328
goal.x =20





goal.y = 0





robot = Robot(10, 50, goal)

robot.setFrequency(20)
while not rospy.is_shutdown():
	#rospy.loginfo(robot.detectCollision())
	#robot.detectGap()
	
	#robot.obstacleAvoidance()
	#robot.setReferencePoint()
	#rospy.loginfo("Lidar difference: " + str(robot.getLidarDifferenceBetweenAngles()))
	#rospy.loginfo(robot.getYaw())
	#rospy.loginfo(robot.getMaxGap())
	#robot.getMaxGap()
	#robot.alignWithGap()
	#rospy.loginfo(robot.distanceToCenterOfGap())
	#robot.moveToCenterOfGap()
	#rospy.loginfo(robot.getLidarTurnDirection())
	robot.moveToGoal()
	#rospy.loginfo(robot.getMaxGap())
	#rospy.loginfo(robot.newWayPoint())
	#rospy.loginfo(robot.distanceToCenterOfGap())
	#robot.distanceToCenterOfGap()
	#rospy.loginfo(robot.newGoalPoint())
	#rospy.loginfo("Angle: " + str(robot.getAngleTowardCenter()))
	#rospy.loginfo(	"Direction: " + str(robot.getLidarTurnDirection()))
	#rospy.loginfo("Old goal: " + str(robot.getGoal()))
	
	#rospy.loginfo("Angle to goal: " + str(robot.getAngleToGoal()))
	#rospy.loginfo("Old goal: " + str(robot.getGoal()))
	#rospy.loginfo("Old turn direction: " + str(robot.getTurnDirection()))
	#newGoal = robot.newGoalPoint()
	#rospy.loginfo(newGoal)
	#robot.setGoal(newGoal)
	#rospy.loginfo("New turn direction: " +str(robot.getTurnDirection()))
	
	#rospy.loginfo("New goal: " + str(robot.getGoal()))
	#rospy.loginfo("New angle to goal: " + str(robot.getAngleToGoal()))
	#rospy.loginfo("Turn direction: " + str(robot.getTurnDirection()))
	#robot.detectObstacle()
	#robot.moveToPosition()
	#robot.moveToCenterOfGap()

