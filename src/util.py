# Grams will be the standard unit.

import time
import usb.core

from boba_bot.srv import *

VENDOR_ID = 0x0922
PRODUCT_ID = 0x8003

SCALE_TOPIC = "scale_readings"
SCALE_NODE = "scale_reader"

SCALE_SERVICE = "scale_readings_service"
SCALE_SERVICE_NODE = "scale_reader_server"

MAIN_NODE = "boba_bot"

CUP_WIDTH = .0975
CUP_HEIGHT = .1

def init_scale():
  """Sets up and returns a (device, endpoint) representing the USB scale.
  """
  device, endpoint = None, None
  for _ in range(1):
    device = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
    if device is not None:
      break
    else:
      print("No scale detected")
      time.sleep(1)
  if device is None:
    print("Giving up; making up fake weights")
  else:
    print("scale found")
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
