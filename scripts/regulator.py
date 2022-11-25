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
		for i in sys.argv:
			print i +"\n"
		if len(sys.argv) > 1:
			self.topicname = sys.argv[1]
		else:
			self.topicname = "/robot0/cmd_vel"	
		self.laserlock = threading.Lock()
		self.pub = rospy.Publisher("/duclos/cmd_vel",Twist,queue_size=10)
		self.sub = rospy.Subscriber("/duclos/speed",Float32,self.speedCallBack)
		self.sub = rospy.Subscriber("/duclos/angular",Float32,self.angularCallBack)
		self.sub = rospy.Subscriber("/duclos/scan",LaserScan,self.laserCallback)
		self.data=None
		rospy.init_node('laserThinker')
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
				self.count += .1
				#acceleration formula over a period of .1 seconds or 10 hz
				self.acceleration = (self.dataSpeed - 0.0)/.1
				if self.count == .1:
					#saves inital accleration for rate over time
					self.initialac = self.acceleration
				print(str(self.acceleration) + "acceleration")
				#jerk formula or the rate of change of acceleration of .1 seconds
				self.jerk = (self.acceleration -self.initialac)/.1
				print(str(self.jerk) + "jerk or acceleration rate of change")
				
				move = Twist()
				

				# I started trying to setup some PID formulas but I didnt want to do calculus at 1 am
				#self.E(t) = 1 -(self.dataSpeed/.1)
				#self.Ed(t) = self.E(t) - self
				#self.p(t) = 1 - (self.dataSpeed/.1)
				#self.pd(t) = (self.p(t) + () + 1 ) * .5
				if self.dataSpeed == 0.0:
					
					move.linear.x = .1
				else:
					move.linear.x = 0
					move.angular.z = 0
				
				self.pub.publish(move)
				
				self.rate.sleep()
				
	
if __name__=='__main__':
	temp = simple_behavior()
	temp.mainloop()
	rospy.spin()

			
