#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
	global range_ahead 
	range_ahead = msg.ranges[len(msg.ranges)/2]
	print(range_ahead)

range_ahead = 0	
rospy.init_node('pub_speed')
scan_sub = rospy.Subscriber('scan',LaserScan,scan_callback)
cmd_vel_pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
speed = Twist()
rate = rospy.Rate(10)
mode = 0

while not rospy.is_shutdown():
	if range_ahead>1:
		speed.linear.x = -0.5
		speed.angular.z = 0
		cmd_vel_pub.publish(speed)
	elif range_ahead<0.5:
                speed.linear.x = 0.5
                speed.angular.z = 0
                cmd_vel_pub.publish(speed)
	else:

		speed.linear.x=0
		speed.angular.z=0.5
		cmd_vel_pub.publish(speed)		
	rate.sleep()	

