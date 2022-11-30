#!/usr/bin/env python

import rospy
import sys
import math
from std_msgs.msg import String
from enum import Enum
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from std_msgs.msg import Float32

# Class to get and store sensor data
class sensorData:
	def __init__(self, name):
		self.roboName = name
		self.hasData = False
		self.speedData = False
		self.angularData = False
		rospy.Subscriber("/"+self.roboName+"/scan", LaserScan, self.callback)
		rospy.Subscriber("/"+self.roboName+"/speed", Float32, self.speedCallback)
		rospy.Subscriber("/"+self.roboName+"/angular", Float32, self.angularCallback)

	def callback(self, data):
		self.dataPack = data
		self.hasData = True

	def speedCallback(self, data):
		self.speedDataPack = data.data
		self.speedData = True

	def angularCallback(self, data):
		self.angularDataPack = data.data
		self.angularData = True

	def getSensorData(self):
		return self.dataPack.ranges

	def getAngleMin(self):
		return self.dataPack.angle_min

	def getAngleMax(self):
		return self.dataPack.angle_max

	def getAngleIncrement(self):
		return self.dataPack.angle_increment

	def getRangeMin(self):
		return self.dataPack.range_min

	def getRangeMax(self):
		return self.dataPack.range_max

	def getSpeed(self):
		return self.speedDataPack

	def getRotation(self):
		return self.angularDataPack

	def isDataAvail(self):
		return self.hasData and self.speedData and self.angularData
	
# class to calculate the next movement of the robot
class mazeFinderState:

	def __init__(self, name):
		self.roboName = name
		self.sensorDataObj = sensorData(self.roboName)
		self.vertPID = PID()
		self.horzPID = PID()

	def calcOpenForce(self, num, theta):
		rawHorz = math.sin(theta) * num
		scaledHorz = math.sin(.5 * math.pi * rawHorz)
		return rawHorz
		
	def calcVert(self, num, theta):
		rawVert = math.cos(theta) * num
		scaledVert = rawVert
		return scaledVert

	def calcTwist(self, myRanges, lenRanges):
		mSpeed = 1
		# get data
		angleIncrement = self.sensorDataObj.getAngleIncrement()
		angleMin = self.sensorDataObj.getAngleMin()
		rangeMax = self.sensorDataObj.getRangeMax()

		# set basic variables
		horzSum = 0
		vertSum = 0
		counter = 0

		length = len(myRanges)
		forwardRange = length/3
		partialForwardRange = forwardRange/2
		middle = length/2
		upperVert = middle + partialForwardRange
		lowerVert = middle - partialForwardRange
		vertMovement = 0
		horzMoveToOpen = 0



		for x in myRanges:
			curRange = x
			if math.isinf(curRange):
				curRange = rangeMax
			theta = counter * angleIncrement + angleMin
			curRange /= rangeMax
			vertMovement += self.calcVert(curRange, theta) # Forward Vector
			horzMoveToOpen += self.calcOpenForce(curRange, theta) # Rotational Vector 1


			counter += 1
		vertMovement /= len(myRanges)
		horzMoveToOpen /= len(myRanges)

		
		rTwist = Twist()

		

		dt = .1 # 10 hz
		self.vertPID = PID()
		self.horzPID = PID()

		actualSpeed = self.sensorDataObj.getSpeed() # change get name
		theoreticalSpeed = vertMovement
		vertMovement = self.vertPID.calc(actualSpeed, theoreticalSpeed)

		rTwist.linear.x = vertMovement


		temp = Twist()
		temp.linear.x = theoreticalSpeed
		temp.angular.z = horzMoveToOpen
		print(temp)


		#rTwist.linear.x = vertMovement * mSpeed
		
		actualRotation = self.sensorDataObj.getRotation() # change get name
		theoreticalRotation = horzMoveToOpen
		horzMovement = self.horzPID.calc(actualRotation, theoreticalRotation)
		rTwist.angular.z = horzMovement
		return rTwist



	def determineMovement(self):
		hasData = self.sensorDataObj.isDataAvail()
		mTwist = Twist()
		if hasData == True:
			sensorRanges = self.sensorDataObj.getSensorData()
			mTwist = self.calcTwist(sensorRanges, len(sensorRanges))
		return mTwist



class PID:
	def __init__(self):
		self.integrator = 0.0
		self.prevError = 0.0
		self.differentiator = 0.0
		self.prevMeasurement = 0.0
		self.out = 0.0
		self.Ki = 1
		self.limMaxInt = 2
		self.limMinInt = -self.limMaxInt
		self.T = .1

	def calc(self, actual, expected):
		p = self.calcError(actual, expected)
		integral = self.calcIntegral(p)
		derivative = self.calcDerivative(p)
		self.prevError = p
		return (p + integral + derivative) * 1

	def calcIntegral(self, error):
		self.integrator = self.integrator + .5 * self.Ki * self.T * (error + self.prevError)

		if self.integrator > self.limMaxInt:
			self.integrator = self.limMaxInt
		elif self.integrator < self.limMinInt:
			self.integrator = self.limMinInt
		return self.integrator

	def calcError(self, actual, expected):
		return expected - actual

	def calcDerivative(self, curError):
		# x = ((change in error)/(change in time))
		changeError = curError - self.prevError
		return changeError/self.T
# class that handles the movement functions
class mazeFinderController:

	def __init__(self, name):
		self.roboName = name


		
		self.pub = rospy.Publisher("/"+str(name)+"/cmd_vel", Twist, queue_size = 10)
		self.rate = rospy.Rate(10)
		self.mazeFinderStateObj = mazeFinderState(name)


	def run(self):
		while not rospy.is_shutdown():

			movementTwist = self.mazeFinderStateObj.determineMovement()
			self.pub.publish(movementTwist)	
			self.rate.sleep()
			

# start of main
if __name__ == '__main__':
	rospy.init_node("Controller", anonymous=True)
	robotName = rospy.get_param("~roboname")

	try:
		mazeFinderObj = mazeFinderController(robotName)
		mazeFinderObj.run()
	except rospy.ROSInterruptException:
		print("ERROR: something happened")
