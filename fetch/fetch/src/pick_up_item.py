#!/usr/bin/env python
import sys, rospy, tf, actionlib, moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler
from look_at_bin import look_at_bin
from std_srvs.srv import Empty
from moveit_msgs.msg import CollisionObject
from moveit_commander import PlanningSceneInterface

if __name__ == '__main__':
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('pick_up_item')
  args = rospy.myargv(argv = sys.argv)
  if len(args) != 2:
    print("usage: pick_up_item.py BIN_NUMBER")
    sys.exit(1)
  item_frame = "item_%d"%int(args[1])

  rospy.wait_for_service("/clear_octomap")
  clear_octomap = rospy.ServiceProxy("/clear_octomap",Empty)

  gripper = actionlib.SimpleActionClient("gripper_controller/gripper_action",\
    GripperCommandAction)
  gripper.wait_for_server() # <1>

  arm = moveit_commander.MoveGroupCommander("arm") # <2>
  arm.allow_replanning(True)
  tf_listener = tf.TransformListener() # <3>
  rate = rospy.Rate(10)

  gripper_goal = GripperCommandGoal() # <4>
  gripper_goal.command.max_effort = 10.0

  scene = PlanningSceneInterface("base_link")

  p = Pose()
  p.position.x = 0.4
  p.position.y = -0.4
  p.position.z = 0.7
  p.orientation = Quaternion(*quaternion_from_euler(0, 1, 1))
  arm.set_pose_target(p) # <5>
  while True:
    if arm.go(True):
      break
    clear_octomap()

  look_at_bin()

  while not rospy.is_shutdown():
    rate.sleep()
    try:
      t = tf_listener.getLatestCommonTime('/base_link', item_frame) # <7>
      if (rospy.Time.now() - t).to_sec() > 0.2:
        rospy.sleep(0.1)
        continue

      (item_translation, item_orientation) = \
        tf_listener.lookupTransform('/base_link', item_frame, t) # <8>
    except(tf.LookupException,tf.Exception,\
           tf.ConnectivityException, tf.ExtrapolationException):
      print("exception!")
      continue

    gripper_goal.command.position = 0.15
    gripper.send_goal(gripper_goal) # <9>
    gripper.wait_for_result(rospy.Duration(1.0))

    print("item: " + str(item_translation))
    p.position.x = item_translation[0]
    p.position.y = item_translation[1]
    p.position.z = item_translation[2] + 0.2
    p.orientation = Quaternion(*quaternion_from_euler(0, 1.2, 0))
    arm.set_pose_target(p)
    while True:
      if arm.go(True):
        break
      clear_octomap()

    print("put down")
    p.position.x = item_translation[0] + 0.25
    p.position.y = item_translation[1] - 0.03
    p.position.z = item_translation[2] + 0.34
    p.orientation = Quaternion(*quaternion_from_euler(0, 1.2, 0))
    arm.set_pose_target(p)
    while True:
      if arm.go(True):
        break
      clear_octomap()
   
    print("catch")
    gripper_goal.command.position = 0
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(2.0))
    while True:
      if arm.go(True):
        break
      clear_octomap()

    rospy.sleep(1)
    print("put up")
    p.position.x = item_translation[0] + 0.24
    p.position.y = item_translation[1] - 0.03
    p.position.z = 0.75
    p.orientation = Quaternion(*quaternion_from_euler(0, 1.2, 0))
    arm.set_pose_target(p)
    while True:
      if arm.go(True):
        break
      clear_octomap()
    
    print("back")
    p.position.x = 0.4
    p.position.y = -0.4
    p.position.z = 0.7
    p.orientation = Quaternion(*quaternion_from_euler(0, 1.2, 0))
    arm.set_pose_target(p)
    while True:
      if arm.go(True):
        break
      clear_octomap()
    rospy.sleep(1)

    p.orientation = Quaternion(*quaternion_from_euler(0, -1.5, 0))
    arm.set_pose_target(p)
    while True:
      if arm.go(True):
        break
      clear_octomap()

    print("in")
    p.position.x = 0.38
    p.position.y = -0.25
    p.position.z = 0.6
    p.orientation = Quaternion(*quaternion_from_euler(0, -1.5, 0))
    arm.set_pose_target(p)
    while True:
      if arm.go(True):
        break
      clear_octomap()
    rospy.sleep(1)

    break # <13>
