#!/usr/bin/python

import math
import cmd
import time
import random
import sys

import tf
import tf2_ros
import exp_quat_func as eqf
import numpy as np

import rospy
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage

import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped

from conf import ingredient_list, menu_list
from util import *
from wrist_movement import save_state, restore_state, rotate_left_wrist, rotate_right_wrist, wrist_setup

import movement

CORRECTED_KEYWORD = "/corrected"

""" Ingredients """

ar_markers = []
ingredients = {}

class Ingredient():
  def __init__(self, name, ar_tag):
    self.name = name
    self.ar_tag = ar_tag

    self.position_head = None
    self.quat_head = None
    self.last_seen_head = None

    self.position_left_arm = None
    self.quat_left_arm = None
    self.last_seen_left_arm = None

    self.position_right_arm = None
    self.quat_right_arm = None
    self.last_seen_right_arm = None

    self.raw = None

def setup_ingredients():
  print ("Setting up ingredients...")
  for ingredient in ingredient_list:
    ingredients[ingredient[0]] = Ingredient(ingredient[0], ingredient[1])
    ar_markers.append(ingredient[1])
    ar_markers.append(ingredient[1]+CORRECTED_KEYWORD)
  print ("Done")

""" AR_TAG METHODS """
def ar_to_ingredient(ar_tag):
  for ingredient in ingredients.values():
    if ingredient.ar_tag == ar_tag or ingredient.ar_tag + CORRECTED_KEYWORD == ar_tag:
      return ingredient
  return None

def tf_callback(data):
  for transform in data.transforms:
    child_frame = transform.child_frame_id
    frame_id = transform.header.frame_id
    if child_frame in ar_markers:
      if frame_id == "/head_camera" and child_frame.find(CORRECTED_KEYWORD) > 0:
        ing = ar_to_ingredient(child_frame)
        if not ing:
          return
        trans = data.transforms[0].transform
        ing.last_seen_head = time.time()
        ing.position_head = (trans.translation.x, trans.translation.y, trans.translation.z)
        ing.quad_head = (trans.rotation.x, trans.rotation.y, trans.rotation.z)
        ing.raw = transform
      if frame_id == "/left_hand_camera":
        ing = ar_to_ingredient(child_frame)
        if not ing:
          return
        trans = data.transforms[0].transform
        ing.last_seen_left_arm = time.time()
        ing.position_left_arm = (trans.translation.x, trans.translation.y, trans.translation.z)
        ing.quad_left_arm = (trans.rotation.x, trans.rotation.y, trans.rotation.z)
      if frame_id == "/right_hand_camera":
        ing = ar_to_ingredient(child_frame)
        if not ing:
          return
        trans = data.transforms[0].transform
        ing.last_seen_right_arm = time.time()
        ing.position_right_arm = (trans.translation.x, trans.translation.y, trans.translation.z)
        ing.quad_right_arm = (trans.rotation.x, trans.rotation.y, trans.rotation.z)

class MainLoop(cmd.Cmd):
  """REPL for executing various Baxter actions.
  typing in "foo" will execute function "do_foo"
  See docs for more detail:
  https://docs.python.org/2/library/cmd.html#cmd.Cmd.precmd
  """

  def preloop(self):
    """Executes only once, before commands are interpreted.
    """
    rospy.init_node(MAIN_NODE)

    rospy.wait_for_service(SCALE_SERVICE)
    self._get_weight = rospy.ServiceProxy(SCALE_SERVICE, get_scale_weight)
    self.get_weight = lambda: self._get_weight().weight.data

    # Setup tf Node
    self.tf = tf.TransformListener()
    rospy.Subscriber("/tf", TFMessage, tf_callback)
    setup_ingredients()

    self.grabbed_cup = None
    self.cup_theta = 0
    self.ingredient_weights = {"nomnom": 400.}

    self.do_setup_motion("")
    self.do_reset("")
    self.do_robot_init("")
    wrist_setup()

  def do_robot_init(self, line):
    #get the head/base transform
    tfBuffer = tf2_ros.Buffer()
    listener2 = tf2_ros.TransformListener(tfBuffer)
    self.listener = tf.TransformListener()
    while True:
      try:
        time.sleep(2)
        #tinme = listener.getLatestCommonTime('base', 'ar_marker_1/corrected')
        #self.trans = tfBuffer.lookup_transform('base', 'ar_marker_1/corrected', tinme, rospy.Duration(12.00000))
        self.trans = tfBuffer.lookup_transform('base', 'head_camera', rospy.Time(0), rospy.Duration(12.0))
        #self.trans = self.listener.lookupTransform('base', 'ar_marker_1/corrected', rospy.Time(0))
        #print self.trans
        rot = self.trans.transform.rotation
        trs = self.trans.transform.translation
        #print rot
        exp = eqf.quaternion_to_exp(np.array([rot.x, rot.y, rot.z, rot.w]))
        #print exp
        rbt = eqf.create_rbt(exp[0], exp[1], np.array([trs.x, trs.y, trs.z]))
        #print rbt
        self.rbt = rbt
        #print dir(listener)
        break
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.sleep(2)

    # self.do_setup_motion()
    # self.movebase()

  def do_setup_motion(self, line=""):
    self.left_arm, self.right_arm = movement.setup_motion()

  def movebase(self):
    #move arm to base position away from cameras
    #base_location = [.22,.88,.5]
    base_location = [.6, 1.2, .34]
    base_plan = self.plan_path(self.left_arm, base_location, [0,0,0])
    while not self.left_arm.execute(base_plan):
      base_plan = self.plan_path(self.left_arm, base_location+[.1*np.random.random(), 0 , .1*np.random.random()], [0,0,0])


  def convert_to_base(self, rbt, ingredient_position):
    x1 = np.array([ingredient_position[i] for i in range(3)] + [1])
    base_coord = self.rbt.dot(x1.reshape(4,1)).getA().flatten()
    return base_coord

  def plan_path(self, grab_arm, base_coord, offset):
    goal_1 = PoseStamped()
    goal_1.header.frame_id = "base"

    #x, y, and z position
    goal_1.pose.position.x = base_coord[0] + offset[0]
    goal_1.pose.position.y = base_coord[1] + offset[1]
    goal_1.pose.position.z = base_coord[2] + offset[2]
    
    #Orientation as a quaternion
    goal_1.pose.orientation.x = 0.0
    goal_1.pose.orientation.y = 2**.5/2
    goal_1.pose.orientation.z = 0.0
    goal_1.pose.orientation.w = 2**.5/2

    #Set the goal state to the pose you just defined
    grab_arm.set_pose_target(goal_1)

    #Set the start state for the left arm
    grab_arm.set_start_state_to_current_state()

    #Plan a path
    grab_plan = grab_arm.plan()
    return grab_plan


  def do_show_head_ingredients(self, line):
    """Displays all ingredients seen by head_camera
    """
    print("Displaying all ingredients that head sees:")
    for ingredient in ingredients.values():
      # If the cup has been seen less than a second ago, consider still visible
      if ingredient.last_seen_head and time.time() - ingredient.last_seen_head < 1:
        print(ingredient.name + " is located at ({}, {}, {})".format(ingredient.position_head[0],
                     ingredient.position_head[1], 
                           ingredient.position_head[2]))  
      else:
        print(ingredient.name + " is not found") 
  
  def do_show_base_ingredients(self, line):
    """Displays all ingredients seen by head_camera wrt base
    """
    print("Displaying all ingredients that head sees:")
    for ingredient in ingredients.values():
      
      # If the cup has been seen less than a second ago, consider still visible
      if ingredient.last_seen_head and time.time() - ingredient.last_seen_head < 1:
        base_coord = self.convert_to_base(self.rbt, ingredient.position_head)
        print base_coord
        print(ingredient.name + " is located at ({}, {}, {})".format(*[base_coord[i] for i in range(3)]))  
      else:
        print(ingredient.name + " is not found")

  def do_show_left_arm_ingredients(self, line):
    """Displays all ingredients seen by left arm_camera
    """
    print("Displaying all ingredients that arm sees:")
    for ingredient in ingredients.values():
      # If the cup has been seen less than a second ago, consider still visible
      if ingredient.last_seen_left_arm and time.time() - ingredient.last_seen_left_arm < 1:
        print(ingredient.name + " is located at ({}, {}, {})".format(ingredient.position_left_arm[0],   
                     ingredient.position_left_arm[1], 
                           ingredient.position_left_arm[2])) 
      else:
        print (ingredient.name + " is not found")

  def do_rotate_wrist(self, line):
    """ rotate_wrist arm delta
        Rotates given wrist by delta (.5~.05)
    """
    args = line.split()
    if(len(args) != 2):
      print("Invalid arguments")
      return
    arm, delta = args
    if arm == "right":
      rotate_right_wrist(float(delta))
    elif arm == "left":
      rotate_left_wrist(float(delta))
    else:
      print("Please enter valid wrist")

  def do_save_wrist(self, line):
    """ save_wrist arm
        remembers current wrist state for restoring later
    """
    if line in ['right', 'left']:
      save_state(line)
    else:
      print("Please enter valid wrist")

  def do_restore_wrist(self, line):
    """ restore_wrist arm delta
        restores wrist to saved state
    """
    if line in ['right', 'left']:
      restore_state(line)
    else:
      print("Please enter valid wrist")

  def do_show_right_arm_ingredients(self, line):
    """Displays all ingredients seen by right arm_camera
    """
    print("Displaying all ingredients that arm sees:")
    for ingredient in ingredients.values():
      # If the cup has been seen less than a second ago, consider still visible
      if ingredient.last_seen_left_arm and time.time() - ingredient.last_seen_left_arm < 1:
        print(ingredient.name + " is located at ({}, {}, {})".format(ingredient.position_right_arm[0],   
                     ingredient.position_right_arm[1], 
                           ingredient.position_right_arm[2])) 
      else:
        print (ingredient.name + " is not found")

  def do_i(self, line):
    exec line

  def do_move(self, line):
    """ move arm x y z
        arm: left or right. Defaults to right.
        x y z: should be kept in range [.2, 1.)
    """
    args = line.split()
    if len(args) == 4:
      arm = args.pop(0)
    else:
      arm = ""

    position = [float(coordinate) for coordinate in args]

    if arm == "left":
      arm = self.left_arm
    else:
      arm = self.right_arm

    movement.move(arm, position)

  def do_reset(self, line):
    """ Resets the given arm to starting position.
        If no arm, or an invalid arm, is given, reset both arms.
        Valid arms are "left" or "right"
    """
    if line != "left":
      self.do_move("right .2 -.6 .2")
    if line != "right":
      self.do_move("left .2 .6 .2")

  def do_forward(self, line):
    """ Moves the given arm forward.
        If no arm, or an invalid arm, is given, move the right arm.
        Valid arms are "left" or "right"
    """
    arm, position = self.get_arm(line)
    if arm == None:
      return
    movement.move_forward(arm, position)

  def do_backward(self, line):
    """ Moves the given arm backwards.
        If no arm, or an invalid arm, is given, move the right arm.
        Valid arms are "left" or "right"
    """
    arm, position = self.get_arm(line)
    if arm == None:
      return
    movement.move_backward(arm, position)

  def do_up(self, line):
    """ Moves the given arm up.
        If no arm, or an invalid arm, is given, move the right arm.
        Valid arms are "left" or "right"
    """
    arm, position = self.get_arm(line)
    if arm == None:
      return
    movement.move_up(arm, position)

  def do_down(self, line):
    """ Moves the given arm down.
        If no arm, or an invalid arm, is given, move the right arm.
        Valid arms are "left" or "right"
    """
    arm, position = self.get_arm(line)
    if arm == None:
      return
    movement.move_down(arm, position)

  def do_out(self, line):
    """ Moves the given arm out.
        If no arm, or an invalid arm, is given, move the right arm.
        Valid arms are "left" or "right"
    """
    arm, position = self.get_arm(line)
    if arm == None:
      return
    movement.move_out(arm, position)

  def do_in(self, line):
    """ Moves the given arm in.
        If no arm, or an invalid arm, is given, move the right arm.
        Valid arms are "left" or "right"
    """
    arm, position = self.get_arm(line)
    if arm == None:
      return
    movement.move_in(arm, position)

  def get_arm(self, line):
    if line == "right":
      arm = self.right_arm
    elif line == "left":
      arm = self.left_arm
    else:
      print("No arm given")
      return None, None
    return arm, self.get_position(line)

  def get_position(self, line):
    return self.tf.lookupTransform(
            "/base",
            "/{}_gripper".format(line),
            rospy.Time(0))[0]

  def do_demo(self, line):
    """ Optional wait_time float argument can be given.
    If given, waits that long before executing the rest of the demo.
    The demo consists of resetting both arms, then moving the right
    arm forwards, then up, then out, then in, then down, then backwards,
    then resetting both arms again.
    """
    if line != "":
      # Pause before executing to get setup for video
      wait_time = float(line)
      rospy.sleep(wait_time)

    arm = "right"
    cmd_list = [self.do_reset, self.do_forward, self.do_up, self.do_out, self.do_in, self.do_down, self.do_backward, self.do_reset]
    for cmd in cmd_list:
      cmd(arm)
      rospy.sleep(1)

  def do_show_menu(self, line):
    """Display available menu"""
    for menu in menu_list:
       print(menu)

  def do_show_ingredient(self, line):
    """Display available ingredients"""
    for ingredient in ingredients.values():
       print(ingredient.name)

  def do_s(self, line):
    self.do_grab("Mango a 0 0 .3")

  def do_grab(self, args):
    """Moves Baxter's gripper to grab a cup.
    """
    cup, arm, x, y, z= args.split(' ')
    x = float(x)
    y = float(y)
    z = float(z)
    #right arm is broken
    grab_arm = self.right_arm
    """
    if arm == "left":
      grab_arm = self.left_arm
    if arm == "right":
      grab_arm = self.right_arm
    """

    if cup != "":
      self.grabbed_cup = cup
      self.report("grabbing")
      print(cup + " is at ")
      base_coord = self.convert_to_base(self.rbt, ingredients[cup].position_head)
      print base_coord
      offset = [x,y,z]
      grab_plan = self.plan_path(grab_arm, base_coord, offset)
      #Execute the plan
      #raw_input('Press <Enter> to move the left arm to goal pose 1 (path constraints are never enforced during this motion): ')
      path_found = grab_arm.execute(grab_plan)
      if not path_found:
        print "try new offset"
        return
    tf_offset = offset
    while True:
      print('down')
      delta = .2
      print tf_offset
      while tf_offset[2] > .1:
        rospy.sleep(1)
        tf_offset[2] -= delta
        print tf_offset
        grab_plan = self.plan_path(grab_arm, base_coord, tf_offset)
        path_found = grab_arm.execute(grab_plan)
        if not path_found:
          print "try new offset"
          return

      print 'up'
      while tf_offset[2] < offset[2]:
        rospy.sleep(1)
        tf_offset[2] += delta
        grab_plan = self.plan_path(grab_arm, base_coord, tf_offset)
        path_found = grab_arm.execute(grab_plan)
        if not path_found:
          print "try new offset"
          return
    else:
      print("Please tell me a cup to grab")

  def do_pour(self, line):
    """Pours x grams of liquid
    """
    try:
      float(line)
    except ValueError:
      print("Invalid argument")
      return
    self.report("pouring")
    print("weight: {}".format(self.get_weight()))
    self.pour(float(line))

  def do_return(self, line):
    """Returns a grabbed cup to its original position.
    """
    self.report("returning")

  def do_release(self, line):
    """Releases a cup and return Baxter to starting position.
    """
    self.grabbed_cup = None
    self.report("releasing")

  def do_print(self, line):
    """Prints the state of the robot.
    """
    for state in ["grabbed_cup", "cup_theta"]:
      print("{}: {}".format(state, getattr(self, state)))

  def do_quit(self, line):
    """End with quit, exit, or ctrl-d
    """
    return self.do_EOF(line)

  def do_exit(self, line):
    """End with quit, exit, or ctrl-d
    """
    return self.do_EOF(line)

  def do_EOF(self, line):
    """End with quit, exit, or ctrl-d
    """
    return True

  def report(self, action):
    """Appends ' cup: {}' to action and then calls format(self.grabbed_cup) and prints
    """
    print((action + ' cup: {}').format(self.grabbed_cup))

  def do_show_scale(self, line):
    """Prints current scale value
    """
    print("Current scale weight is {} grams".format(self.get_weight()))

  # Sleep for x seconds in increments of 10 ms, unless there is has been weight change of more than 5 grams
  def sleep_and_measure(self, sleep_duration, threshold=2):
    start = time.time()
    last_weight = self.get_weight()
    while(abs(time.time() - start) < sleep_duration):
      current = self.get_weight()
      print("current {} last {}".format(current, last_weight))
      if abs(last_weight - current) >= threshold:
        print("Poured more than 2 grams")
        self.do_rotate_wrist("right {}".format(.1))
        time.sleep(1.5)
        return True
      last_weight = current
      time.sleep(0.01)
    return False

  def pour(self, weight=60, delta=5.):
    self.do_save_wrist("right")
    total_weight = weight
    current = self.get_weight()
    last_weight = current
    target = current + total_weight
    diff = target - current
    max_count = 50
    inc = -.05
    sleep_duration = .5
    last_change = 0
    while diff > delta and max_count > 0:
      #diff_percent = .8 * diff / total_weight
      #diff_theta = math.atan(2 * CUP_HEIGHT * diff_percent / CUP_WIDTH)
      # Rotate cup by theta
      print("Rotating")
      self.do_rotate_wrist("right {}".format(inc))
      #print("increasing theta by: {}".format(diff_theta))
      #self.cup_theta += diff_theta

      current = self.get_weight()
      diff = target - current
      # If there's less than 10% of goal to go, slow down
      if abs(target-current) < total_weight * .10:
        inc = -.02
        sleep_duration = 1
        print("I am almost there")
      # If there's been any changes more than 5 grams, pull back then slow down 
      if(self.sleep_and_measure(sleep_duration)):
        sleep_duration = 1
        inc = -.02
      max_count -= 1

    # reset cup orientation
    self.cup_theta = 0
    self.do_restore_wrist("right")

if __name__ == '__main__':
  MainLoop().cmdloop()
