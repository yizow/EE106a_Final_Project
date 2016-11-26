import usb.core
import usb.util

PYUSB_DEBUG=True

VENDOR_ID = 0x0922
PRODUCT_ID = 0x8003

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

while True:
    try:
        data = device.read(endpoint.bEndpointAddress,
                           endpoint.wMaxPacketSize)
        print data
    except usb.core.USBError as e:
        data = None
        if e.args == ('Operation timed out',):
            attempts -= 1
            continue

# Returns 6 bytes
# 0: always 3
# 1: 2 if 0, 4 otherwise
# 2: 2 if lbs/oz, 11 if kg/g
# 3: Signed int. Scaling factor in terms of powers of 10. 0 means multiply by 1, 255 = -1 means multiply by .1. Made redundant by Byte 2
# 4: Value. Either ounces, or grams. Dont forget to scale. 
# 5: Value. Higher byte. Total weight is Byte 5 * 256 + Byte 4
