#!/usr/bin/env python
import os, sys, rospy, tf, actionlib, moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from geometry_msgs.msg import *
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('pick_up_item')

  rospy.wait_for_service("/clear_octomap")
  clear_octomap = rospy.ServiceProxy("/clear_octomap",Empty)

  gripper = actionlib.SimpleActionClient("gripper_controller/gripper_action",
    GripperCommandAction)
  gripper.wait_for_server() # <1>

  arm = moveit_commander.MoveGroupCommander("arm") # <2>
  arm.allow_replanning(True)
  tf_listener = tf.TransformListener() # <3>
  rate = rospy.Rate(10)

  gripper_goal = GripperCommandGoal() # <4>
  gripper_goal.command.max_effort = 10.0

  p = Pose()
  p.position.x = 0.4
  p.position.y = -0.4
  p.position.z = 0.7
  p.orientation = Quaternion(*quaternion_from_euler(0, -1.5, 0))
  arm.set_pose_target(p) # <5>
  while True:
    if arm.go(True):
      break
    clear_octomap()

  p.orientation = Quaternion(*quaternion_from_euler(0, 1.2, 0))
  arm.set_pose_target(p)
  while True:
    if arm.go(True):
      break
    clear_octomap()

  p.position.x = 0.4
  p.position.y = 0
  p.position.z = 0.95
  p.orientation = Quaternion(*quaternion_from_euler(0, 1.2, 0))
  arm.set_pose_target(p) # <5>
  while True:
    if arm.go(True):
      break
    clear_octomap()

  p.position.x = 0.75
  p.position.y = 0
  p.position.z = 0.95
  p.orientation = Quaternion(*quaternion_from_euler(0, 1.2, 0))
  arm.set_pose_target(p) # <5>
  while True:
    if arm.go(True):
      break
    clear_octomap()

  p.position.x = 0.9
  p.position.y = 0
  p.position.z = 0.95
  p.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
  arm.set_pose_target(p) # <5>
  while True:
    if arm.go(True):
      break
    clear_octomap()

  print("open")
  gripper_goal.command.position = 10
  gripper.send_goal(gripper_goal)
  gripper.wait_for_result(rospy.Duration(2.0))
  while True:
    if arm.go(True):
      break
    clear_octomap()

