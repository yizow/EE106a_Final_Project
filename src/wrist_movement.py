#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# We used the examples joint_position_keyboard.py and gripper_keyboard from baxter examples 
# to help model these functions
import time
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

saved_state = {'right_w2': None, 'left_w2': None}
ERROR = 0.02
TIMEOUT = 5  
W2_INDEX = 6
left_limb = None
right_limb = None
left_gripper = None
right_gripper = None

""" Wrist Rotation Functions """

# Stores limb's wrist state in variable
def save_state(limb_name):
  if limb_name == 'right':
    limb = right_limb
    wrist_name = 'right_w2'
  elif limb_name == 'left':
    limb = left_limb
    wrist_name = 'left_w2'
  else:
    print("Please enter valid name")
    return 
  saved_state[wrist_name] = limb.joint_angles()[wrist_name]
  print("{} saved to {}".format(limb_name, saved_state[wrist_name]))

# Restores limb's wrist state until timeout or sufficient delta
def restore_state(limb_name):
  if limb_name == 'right':
    limb = right_limb
    wrist_name = 'right_w2'
  elif limb_name == 'left':
    limb = left_limb
    wrist_name = 'left_w2'
  else:
    print("Please enter valid name")
    return  
  
  if(not saved_state[wrist_name]):
    print("{} has no saved state".format(limb_name))
  joints = limb.joint_names()
  delta = abs(limb.joint_angles()[wrist_name] - saved_state[wrist_name])
  start_time = time.time()
  while delta > ERROR and time.time() - start_time < TIMEOUT:
    limb.set_joint_positions({wrist_name: saved_state[wrist_name]})
    delta = abs(limb.joint_angles()[wrist_name] - saved_state[wrist_name])

# Rotates wrist by delta
def rotate_wrist(wrist_name, delta):
  if (wrist_name == 'right'):
    limb = right_limb
  elif (wrist_name == 'left'):
    limb = left_limb
  else:
    print("No limb {} ".format(wrist_name))
    return
  joint_name = limb.joint_names()[6]
  current_position = limb.joint_angle(joint_name)
  joint_command = {joint_name: current_position + delta }
  limb.set_joint_positions(joint_command)
  print(joint_command)


""" Gripper Functions """

# Open, calibrate, then close
def g_grab(gripper_name):
  if(gripper_name in ['right', 'left']):
    g_open(gripper_name)
    g_calibrate(gripper_name)
    g_close(gripper_name)
  else:
    print("No {} gripper".format(gripper_name))

# Calibrate the gripper
def g_calibrate(gripper_name):
  if(gripper_name == 'right'):
    right_gripper.calibrate()
  elif(gripper_name == 'left'):
    left_gripper.calibrate()
  else:
    print("No {} gripper".format(gripper_name))

# Open the Gripper
def g_open(gripper_name):
  if(gripper_name == 'right'):
    right_gripper.open()
  elif(gripper_name == 'left'):
    left_gripper.open()
  else:
    print("No {} gripper".format(gripper_name))

# Close the Gripper
def g_close(gripper_name):
  if(gripper_name == 'right'):
    right_gripper.close()
  elif(gripper_name == 'left'):
    left_gripper.close()
  else:
    print("No {} gripper".format(gripper_name))

def offset_holding(gripper, offset):
  if gripper.type() != 'electric':
    return
  current = gripper.parameters()['holding_force']
  gripper.set_holding_force(current + offset)

# Adjust holding strength by val
def g_adj_hold(gripper_name, val):
  if(gripper_name == 'right'):
    offset_holding(right_gripper, val) 
  elif(gripper_name == 'left'):
    offset_holding(left_gripper, val) 
  else:
    print("No {} gripper".format(gripper_name))

""" Setup global interfaces """
def wrist_setup():
  global left_limb, right_limb, left_gripper, right_gripper
  left_limb = baxter_interface.Limb('left')
  right_limb = baxter_interface.Limb('right')
  left_gripper = baxter_interface.Gripper('left')
  right_gripper = baxter_interface.Gripper('right')
