#!/usr/bin/env python

import time
import usb.core

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
  def read_scale_helper(req):
    if device is None:
      return get_scale_weightResponse(Float32(3.14159))
    try:
      data = device.read(endpoint.bEndpointAddress,
                         endpoint.wMaxPacketSize)
      return get_scale_weightResponse(parse_scale(data))
    except usb.core.USBError as e:
      data = None
      if e.args == ('Operation timed out',):
        print e.args
        raise e
  return read_scale_helper

def parse_scale(reading):
  """Converts a scale reading to a float representing the weight in grams.
  """
  value = reading[4] + 256. * reading[5]
  ounce_mode = reading[2] == 11
  if ounce_mode:
    value *= 2.83495
  # Scale can only achieve .1 gram precision
  return Float32(value * 10 // 10)

def main():
  device, endpoint = init_scale()

  server = rospy.Service(SCALE_SERVICE, get_scale_weight, read_scale(device, endpoint))
  rospy.init_node(SCALE_SERVICE_NODE)

  print("Ready to service")
  rospy.spin()

if __name__ == '__main__':
  main()
