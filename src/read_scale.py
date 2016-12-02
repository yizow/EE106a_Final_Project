#!/usr/bin/env python

import time
import usb.core
import usb.util

import rospy
from std_msgs.msg import Float32

from util import *

def read_scale(device, endpoint):
  """Returns 6 bytes read from scale. If scale is unplugged, returns None.
  Byte: Description
  0: always 3
  1: 2 if scale empty (0 reading), 4 otherwise
  2: 11 if lbs/oz, 2 if kg/g
  3: Signed int. Scaling factor in terms of powers of 10. 0 means multiply by 1, 255 = -1 means multiply by .1. Made redundant by Byte 2
  4: Value. Either ounces, or grams. Dont forget to scale.
  5: Value. Higher byte. Total weight is Byte 5 * 256 + Byte 4
  """
  try:
    data = device.read(endpoint.bEndpointAddress,
                       endpoint.wMaxPacketSize)
    return data
  except usb.core.USBError as e:
    data = None
    if e.args == ('Operation timed out',):
      print("s")
      print e.args
      raise e

def parse_scale(reading):
  """Converts a scale reading to a float representing the weight in grams.
  """
  value = reading[4] + 256. * reading[5]
  ounce_mode = reading[2] == 11
  if ounce_mode:
    value *= 2.83495
  # Scale can only achieve .1 gram precision
  return value * 10 // 10

def main():
  pub = rospy.Publisher(SCALE_TOPIC, Float32, queue_size=10)
  rospy.init_node(SCALE_NODE)

  device, endpoint = init_scale()

  # We're not rate limiting since USB protocol itself is rate limited
  while not rospy.is_shutdown():
    data = read_scale(device, endpoint)
    if data is not None:
      pub.publish(parse_scale(data))


if __name__ == '__main__':
  main()
