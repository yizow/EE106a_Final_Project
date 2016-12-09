import sys

import numpy as np

import rospy

import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped

DELTA = .2

X_MAX = .95
X_MIN = .2
Z_MAX = .95
Z_MIN = .2

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

def move_forward(arm, start):
  x, y, z = start
  x_points = list(np.arange(x, X_MAX, DELTA))
  steps = [(x, y, z) for x in x_points]
  # Make sure we end at a consistent location
  if steps[-1][0] != X_MAX:
    steps.append((X_MAX, y, z))

  move_steps(arm, steps)

def move_backward(arm, start):
  x, y, z = start
  x_points = list(np.arange(x, X_MIN, -DELTA))
  steps = [(x, y, z) for x in x_points]
  # Make sure we end at a consistent location
  if steps[-1][0] != X_MIN:
    steps.append((X_MIN, y, z))

  move_steps(arm, steps)

def move_up(arm, start):
  x, y, z = start
  z_points = list(np.arange(z, Z_MAX, DELTA))
  steps = [(x, y, z) for z in z_points]
  # Make sure we end at a consistent location
  if steps[-1][0] != Z_MAX:
    steps.append((x, y, Z_MAX))

  move_steps(arm, steps)

def move_down(arm, start):
  x, y, z = start
  z_points = list(np.arange(z, Z_MIN, -DELTA))
  steps = [(x, y, z) for z in z_points]
  # Make sure we end at a consistent location
  if steps[-1][0] != Z_MIN:
    steps.append((x, y, Z_MIN))

  move_steps(arm, steps)

def move_steps(arm, steps):
  for step in steps:
    print "moving to: {}".format(step)
    move(arm, step, True)

def move(arm, destination, constrained=False):
  goal = create_goal(destination)
  set_goal_orientation(goal.pose)

  #Set the goal state to the pose you just defined
  arm.set_pose_target(goal)

  #Set the start state for the left arm
  arm.set_start_state_to_current_state()

  # Constrain gripper to always point forwards
  if constrained:
    arm.set_path_constraints(create_constraint(arm.get_end_effector_link()))

  #Plan a path
  plan = arm.plan()

  arm.execute(plan)
