#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class Stopper:
	vel = Twist()
	keepMoving = True
	isObstacleInFront = False
	FORWARD_SPEED = 0.3
	DIST_FROM_OBSTACLE = 0.3
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	def __init__(self):
		pass

	def state(self):
		rospy.init_node('walker', anonymous=False)
		
		sub = rospy.Subscriber('scan', LaserScan, self.callback)
		rate = rospy.Rate(10) # 10hz
		self.keepMoving = True
		while not rospy.is_shutdown():
			if self.keepMoving:
				self.pub.publish(self.vel)
				rate.sleep()
			else:
				self.pub.publish(self.vel)
				rate.sleep()

	def move_forward(self):
		self.vel.angular.z = 0
		self.vel.linear.x = (-1)*self.FORWARD_SPEED
		self.keepMoving = True
		self.pub.publish(self.vel)


	def find_free_way(self, msg):
		minLeft = 240
		maxLeft = 300
		minRight = 60
		maxRight = 120
		leftInfSum = 0
		rightInfSum = 0
		rearInfSum = 0
		leftSum = 0
		rightSum = 0
		rearSum = 0

		k = 0
		while minRight + k < maxRight:
			k+=1
			if msg.ranges[minRight + k] == math.inf:
				rightInfSum += 1

			if msg.ranges[maxLeft - k] == math.inf:
				leftInfSum += 1
			
			if k < 30:
				if msg.ranges[30 - k] == math.inf:
					rearInfSum += 1
				if msg.ranges[330 + k] == math.inf:
					rearInfSum += 1

			if msg.ranges[minRight + k] != math.inf:
				rightSum += msg.ranges[minRight + k]

			if msg.ranges[maxLeft - k] != math.inf:
				leftSum += msg.ranges[maxLeft - k]
			
			if k < 30:
				if msg.ranges[30 - k] != math.inf:
					rearSum += msg.ranges[30 - k]
				if msg.ranges[330 + k] != math.inf:
					rearSum += msg.ranges[330 + k]


		

		self.vel.linear.x = 0
		if rightInfSum != leftInfSum:
			if rearInfSum > rightInfSum & rearInfSum > leftInfSum:
				self.vel.linear.x = 1*self.FORWARD_SPEED
			if rightInfSum < leftInfSum: 
				self.vel.angular.z = 1
			if rightInfSum > leftInfSum: 
				self.vel.angular.z = -1
		else: 
			if rightSum != leftSum:
				if rightSum < leftSum:
					self.vel.angular.z = 1
				if rightSum > leftSum:
					self.vel.angular.z = -1
			else:
				self.vel.linear.x = 1*self.FORWARD_SPEED
		self.keepMoving = True
		self.pub.publish(self.vel)

	def checking_for_obstacles(self, msg):
		if self.isObstacleInFront:
			self.find_free_way(msg)
		else:
			self.move_forward()

	def front_scan(self, msg):
		minIndex = 135
		maxIndex = 225
		midIndex = 180
		k = 1
		while minIndex + k != midIndex:
			if msg.ranges[int(minIndex + k)] < self.DIST_FROM_OBSTACLE:
				self.isObstacleInFront = True
				break
			elif msg.ranges[int(maxIndex - k)] < self.DIST_FROM_OBSTACLE:
				self.isObstacleInFront = True
				break
			else:
				self.isObstacleInFront = False
			k += 1
		self.checking_for_obstacles(msg)

	def callback(self, msg):
		self.front_scan(msg)
			

if __name__ == '__main__':
	try:
		st = Stopper()
		st.state()
	except rospy.ROSInterruptException:
		pass