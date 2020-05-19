#!/usr/bin/env python
# coding=utf-8
import math, time
import rospy
from std_msgs.msg import Float64

rospy.init_node('sine_wave')#初始化节点
pub = rospy.Publisher('sin', Float64,queue_size = 1)#发布sin话题，消息类型是Float64
while not rospy.is_shutdown():#一直发布
  msg = Float64()#创建Float64对象
  msg.data = math.sin(4*time.time())#对象的数据
  pub.publish(msg)#发布消息
  time.sleep(0.1)
