#!/usr/bin/env python
from unitysim.msg import BoundingBox3d
import sys
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import threading
class simple_behavior:
	def __init__(self):
		#for i in sys.argv:
		#	print i +"\n"
		#if len(sys.argv) > 1:
		#	self.topicname = sys.argv[1]
		#else:
		#	self.topicname = "/robot0/cmd_vel"
		rospy.init_node('laserThinker')
		robotName = rospy.get_param("~roboname")
			
		self.laserlock = threading.Lock()


		self.pub = rospy.Publisher("/" +robotName+"/cmd_vel",Twist,queue_size=10)
		rospy.Subscriber("/"+robotName+"/speed",Float32,self.speedCallBack)
		rospy.Subscriber("/"+robotName+"/angular",Float32,self.angularCallBack)
		rospy.Subscriber("/"+robotName+"/scan",LaserScan,self.laserCallback)

		self.data=None

		self.rate = rospy.Rate(10) # 10hz WE KNOW TEN HZ!
		self.ranges=[]
		self.command = ""
		self.commandcallback =""
		self.state = ""
		self.dataSpeed =0.0
		self.dataAngular =0.0
		self.count = 0
		self.initialac = 0


	def speedCallBack(self,data):
		
		self.dataSpeed=data.data
		
	def angularCallBack(self,data):
		
		self.dataAngular=data.data
	def laserCallback(self,data):
	
		self.data=data
		
	def mainloop(self):
		#I didn't finish the script because it turns out the topics are quite confusing. Right now it just runs some basic accleration calculations. The bot accelerates at .5 units per second
		#What I have not figured out is how the acceleration interacts with a twist value. For example if if you run the code as it is setup right now after waiting enough time a linear x of 0 will eventually cause the robot to slowly move backward (or atleast it should having tested it multiple times) The robot requires some sort of input to start automoving / turning at all. 
		while not rospy.is_shutdown():
				#counter to represent time
				dt = .1
				self.count += dt
				#acceleration formula over a period of .1 seconds or 10 hz
				
				move = Twist()
				# I started trying to setup some PID formulas but I didnt want to do calculus at 1 am
				#self.E(t) = 1 -(self.dataSpeed/.1)
				#self.Ed(t) = self.E(t) - self
				#self.p(t) = 1 - (self.dataSpeed/.1)
				#self.pd(t) = (self.p(t) + () + 1 ) * .5
				#if self.dataSpeed == 0.0:
					
				#	move.linear.x = 10 * (1+self.count)
				#else:
				move.linear.x = 10 * (1+self.count)
				move.angular.z = 0
				
				self.pub.publish(move)
				
				self.rate.sleep()
				
	def pController(self, curOutput, expectedOutput, defaultScalar):
		return 1-(curOutput/expectedOutput)+defaultScalar # its a start

	def pdController(self, curOutput, expectedOutput, oldCur, oldExpected,dt, scale):
		# ([1-(actual/desired)] + ([e(t)-e(t-1)])]/dt)+1)*input
		p = 1 - (curOutput/expectedOutput)
		et = p
		etOld = 1 - (oldCur/oldExpected)
		return (p +((et-etOld)/dt)+1)*scale # this is a start def wrong

	def pdiController(self, timeWindow, etList, initialInput):
		# i(t) = (1/timewindow)(**sum**[E(T)]+initialInput)
		etSum = 0
		for x in etList:
			etSum += x
		return (1/timewindow)(etSum+initialInput) # might be wrong have not tested it but a start
		
if __name__=='__main__':
	temp = simple_behavior()
	temp.mainloop()
	rospy.spin()

			
