#!/usr/bin/python

import cmd

import rospy
from std_msgs.msg import Float32

from util import *

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
    self.grabbed_cup = None

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
