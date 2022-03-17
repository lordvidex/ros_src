#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys

def pose_callback(pose):
	rospy.loginfo("Robot X=%f : Y=%f : Z=%f\n", pose.x, pose.y, pose.theta)

def move_turtle():
	rospy.init_node('move_turtle', anonymous=False)
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber('/turtle1/pose',Pose, pose_callback)
	rate = rospy.Rate(10) # 10hz
	lin_vel = 1.0
	ang_vel = 0.5
	vel = Twist()
	vel.linear.x = lin_vel
	vel.linear.y = 0
	vel.linear.z = 0
	vel.angular.x = 0
	vel.angular.y = 0
	vel.angular.z = ang_vel
	rospy.loginfo("LinearVel=%f: AngularVel=%f", lin_vel, ang_vel)
	while not rospy.is_shutdown():
		pub.publish(vel)
		rate.sleep()

if __name__ == '__main__':
	try:
		move_turtle()
	except rospy.ROSInterruptException:
		pass
