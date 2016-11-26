#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import usb.core
import usb.util

VENDOR_ID = 0x0922
PRODUCT_ID = 0x8003

def init_scale():
  """Sets up and returns a (device, endpoint) representing the USB scale.
  """
  device = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)

  for cfg in device:
    for intf in cfg:
      if device.is_kernel_driver_active(intf.bInterfaceNumber):
        try:
          device.detach_kernel_driver(intf.bInterfaceNumber)
        except usb.core.USBError as e:
          sys.exit("Could not detatch kernel driver from interface({0}): {1}".format(intf.bInterfaceNumber, str(e)))

  device.set_configuration()

  endpoint = device[0][(0, 0)][0]

  return device, endpoint


def read_scale(device, endpoint):
  """ Returns 6 bytes read from scale
  Byte: Description
  0: always 3
  1: 2 if 0, 4 otherwise
  2: 2 if lbs/oz, 11 if kg/g
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
      print e.args
      raise e

def main():
  pub = rospy.Publisher('scale_readings', String, queue_size=10)
  rospy.init_node('scale_reader', anonymous=True)

  device, endpoint = init_scale()

  # We're not rate limiting since USB protocol itself is rate limited
  while not rospy.is_shutdown():
    data = str(read_scale(device, endpoint))
    rospy.loginfo(data)
    pub.publish(data)


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
