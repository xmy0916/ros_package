#!/usr/bin/env python
import os, sys, rospy, tf, actionlib
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from look_at_bin import look_at_bin
import moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
import sys, rospy, tf, actionlib, moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler
from look_at_bin import look_at_bin
from std_srvs.srv import Empty
from moveit_msgs.msg import CollisionObject
from moveit_commander import PlanningSceneInterface

if __name__ == '__main__':
  os.system("rosservice call /move_base/clear_costmaps")
  rospy.init_node('go_to_bin')
  args = rospy.myargv(argv=sys.argv)
  if len(args) != 2:
    print "usage: go_to_bin.py BIN_NUMBER"
    sys.exit(1)
  bin_number = int(args[1])
  move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  move_base.wait_for_server()
  goal = MoveBaseGoal()
  goal.target_pose.header.frame_id = 'map'
  clear_octomap = rospy.ServiceProxy("/clear_octomap",Empty)
  if bin_number == 100:
    goal.target_pose.pose.position.x = 3.75
    goal.target_pose.pose.position.y = 0
  else:
    goal.target_pose.pose.position.x = 0.5 * (bin_number % 6) - 1.5
    goal.target_pose.pose.position.y = 1.1 * (bin_number / 6) - 0.55

  if bin_number < 6:
    yaw = -1.57
  elif bin_number == 100:
    yaw = 0
  else:
    yaw = 1.57

  orient = Quaternion(*quaternion_from_euler(0, 0, yaw))
  goal.target_pose.pose.orientation = orient
  move_base.send_goal(goal)
  move_base.wait_for_result()
  for i in range(2):
  	look_at_bin()
