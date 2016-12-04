#!/usr/bin/python

import cmd
import time 
import rospy

from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
from conf import ingredient_list, menu_list
from util import *
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
      if frame_id == "/left_hand_camera":     
        ing = ar_to_ingredient(child_frame)
        if not ing:
  	  return 
        trans = data.transforms[0].transform
        ing.last_seen_left_arm = time.time()
        ing.position_left_arm = (trans.translation.x, trans.translation.y, trans.translation.z)
        ing.quad_left_arm = (trans.rotation.x, trans.rotation.y, trans.rotation.z)

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
    self.get_weight = rospy.ServiceProxy(SCALE_SERVICE, get_scale_weight)

    # Setup tf Node
    rospy.Subscriber("/tf", TFMessage, tf_callback)
    setup_ingredients()
    self.grabbed_cup = None
  
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

  def do_show_arm_ingredients(self, line):
    """Displays all ingredients seen by arm_camera
    """
    print("Displaying all ingredients that arm sees:")
    for ingredient in ingredients.values():
      # If the cup has been seen less than a second ago, consider still visible
      if ingredient.last_seen_left_arm:# and time.time() - ingredient.last_seen_left_arm < 1:
        print(ingredient.name + " is located at ({}, {}, {})".format(ingredient.position_left_arm[0],   
								     ingredient.position_left_arm[1], 
							             ingredient.position_left_arm[2])) 
      else:
        print (ingredient.name + " is not found")

  def do_show_menu(self, line):
    """Display available menu"""
    for menu in menu_list:
       print(menu)

  def do_show_ingredient(self, line):
    """Display available ingredients"""
    for ingredient in ingredients.values():
       print(ingredient.name)

  def do_grab(self, cup):
    """Moves Baxter's gripper to grab a cup.
    """
    if cup != "":
      self.grabbed_cup = cup
      self.report("grabbing")
    else:
      print("Please tell me a cup to grab")

  def do_move(self, line):
    """Moves a grabbed cup to pouring position.
    """
    self.report("moving")

  def do_pour(self, line):
    """Pours a grabbed cup.
    """
    self.report("pouring")
    response = self.get_weight()
    print("weight: {}".format(response.weight.data))

  def do_return(self, line):
    """Returns a grabbed cup to its original position.
    """
    self.report("returning")

  def do_release(self, line):
    """Releases a cup and return Baxter to starting position.
    """
    self.grabbed_cup = None
    self.report("releasing")

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

if __name__ == '__main__':
  MainLoop().cmdloop()
