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

# We use this code from joint_position_keyboard.py from baxter examples to help control 
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

# Global Limbs
def set_j(limb, joint_name, delta):
  current_position = limb.joint_angle(joint_name)
  joint_command = {joint_name: current_position + delta }
  limb.set_joint_positions(joint_command)
  print(joint_command)

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

def rotate_left_wrist(delta):
  lj = left_limb.joint_names()
  set_j(left_limb, lj[6], delta)

def rotate_right_wrist(delta):
  rj = right_limb.joint_names()
  set_j(right_limb, rj[6], delta)

def wrist_setup():
  global left_limb
  global right_limb
  left_limb = baxter_interface.Limb('left')
  right_limb = baxter_interface.Limb('right')
