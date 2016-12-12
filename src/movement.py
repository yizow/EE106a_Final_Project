import sys
import math

import numpy as np

import rospy

import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped

DELTA = .1

X_MAX = 1.1
X_MIN = .2
Z_MAX = .4
Z_MIN = .14
# Y_MAX is the absolute value. The right arm uses negative y values
Y_MAX = .8
Y_MIN = .3

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
  orien_const.weight = 3.0;
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
  if steps[-1][2] != Z_MAX:
    steps.append((x, y, Z_MAX))

  move_steps(arm, steps)

def move_down(arm, start):
  x, y, z = start
  z_points = list(np.arange(z, Z_MIN, -DELTA))
  steps = [(x, y, z) for z in z_points]
  # Make sure we end at a consistent location
  if steps[-1][2] != Z_MIN:
    steps.append((x, y, Z_MIN))

  move_steps(arm, steps)

def move_out(arm, start):
  x, y, z = start
  maximum = math.copysign(Y_MAX, y)
  y_points = list(np.arange(y, maximum, math.copysign(DELTA, y)))
  steps = [(x, y, z) for y in y_points]
  # Make sure we end at a consistent location
  if steps[-1][1] != maximum:
    steps.append((x, maximum, z))

  move_steps(arm, steps)

def move_in(arm, start):
  x, y, z = start
  minimum = math.copysign(Y_MIN, y)
  y_points = list(np.arange(y, minimum, math.copysign(DELTA, y)))
  steps = [(x, y, z) for y in y_points]
  # Make sure we end at a consistent location
  if steps[-1][1] != minimum:
    steps.append((x, minimum, z))

  move_steps(arm, steps)

def move_steps(arm, steps):
  for step in steps:
    print "moving to: {}".format(step)
    move(arm, step, True)

def move_steps_axis(arm, start, destination, axis):
  destination = destination.split()
  destination = list([float(_) for _ in destination])
  def overwrite_axis(value):
    new_position = list(start)
    new_position[index] = value
    return new_position

  axis_mapping = {'x': 0, 'y': 1, 'z': 2}
  index = axis_mapping[axis]
  x, y, z = start
  delta = math.copysign(DELTA, destination[index] - start[index])

  axis_points = list(np.arange(start[index], destination[index], delta))
  steps = [overwrite_axis(point) for point in axis_points]
  if len(steps) == 0:
    steps = [overwrite_axis(destination[index])]

  move_steps(arm, steps)
  return steps[-1]

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

  if constrained:
    arm.clear_path_constraints()

def rotate(arm, destination):
  goal = create_goal(destination)
  goal.pose.orientation.x -= .5
  goal.pose.orientation.z -= .5

  set_goal_orientation(goal.pose)

  #Set the goal state to the pose you just defined
  arm.set_pose_target(goal)

  #Set the start state for the left arm
  arm.set_start_state_to_current_state()

  # Constrain gripper to always point forwards

  #Plan a path
  plan = arm.plan()

  arm.execute(plan)