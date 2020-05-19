#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
 
cmd_vel_pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
rospy.init_node('red_light_green_light')
 
red_light_twist=Twist()
green_light_twist=Twist()
deriving_forward=False
 
 
green_light_twist.linear.x=0.5
red_light_twist.linear.x=-0.5
light_change_time=rospy.Time.now()+rospy.Duration(3)
rate=rospy.Rate(10)
 
while not rospy.is_shutdown():
    if deriving_forward :
        cmd_vel_pub.publish(green_light_twist)
    else:
        cmd_vel_pub.publish(red_light_twist)
        
    if light_change_time < rospy.Time.now():
        deriving_forward= not deriving_forward
        light_change_time= rospy.Time.now()+rospy.Duration(3)
    rate.sleep()



