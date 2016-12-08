import sys

import numpy as np

import rospy

import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped

DELTA = .1

def setup_motion():
  moveit_commander.roscpp_initialize(sys.argv)
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  left_arm = moveit_commander.MoveGroupCommander('left_arm')
  right_arm = moveit_commander.MoveGroupCommander('right_arm')
  left_arm.set_planner_id('RRTConnectkConfigDefault')
  left_arm.set_planning_time(5)
  right_arm.set_planner_id('RRTConnectkConfigDefault')
  right_arm.set_planning_time(5)
  return left_arm, right_arm

def create_goal(end_position):
  goal = PoseStamped()
  goal.header.frame_id = "base"

  goal.pose.position.x = end_position[0]
  goal.pose.position.y = end_position[1]
  goal.pose.position.z = end_position[2]

  set_goal_orientation(goal.pose)

  return goal

def set_goal_orientation(goal):
  goal.orientation.x = 0.0
  goal.orientation.y = 2**.5/2
  goal.orientation.z = 0.0
  goal.orientation.w = 2**.5/2

def create_constraint(name):
  orien_const = OrientationConstraint()
  orien_const.link_name = name;
  orien_const.header.frame_id = "base";
  orien_const.orientation.y = 2**.5/2
  orien_const.orientation.w = 2**.5/2
  orien_const.absolute_x_axis_tolerance = 0.1;
  orien_const.absolute_y_axis_tolerance = 0.1;
  orien_const.absolute_z_axis_tolerance = 0.1;
  orien_const.weight = 1.0;
  consts = Constraints()
  consts.orientation_constraints = [orien_const]
  return consts

def move_forward(arm, position):
  x, y, z = position
  x_points = list(np.arange(x, .95, DELTA))
  steps = [(x, y, z) for x in x_points]
  # Make sure we end at a consistent location
  if steps[-1][0] != .95:
    steps.append((.95, y, z))

  move_steps(arm, steps)

def move_backward(arm, position):
  x, y, z = position
  x_points = list(np.arange(x, .2, -DELTA))
  steps = [(x, y, z) for x in x_points]
  # Make sure we end at a consistent location
  if steps[-1][0] != .2:
    steps.append((.2, y, z))

  move_steps(arm, steps)

def move_steps(arm, steps):
  for step in steps:
    print "moving to: {}".format(step)
    move(arm, step)

def move(arm, position):
  goal = create_goal(position)
  set_goal_orientation(goal.pose)

  #Set the goal state to the pose you just defined
  arm.set_pose_target(goal)

  #Set the start state for the left arm
  arm.set_start_state_to_current_state()

  #Plan a path
  plan = arm.plan()

  arm.execute(plan)
