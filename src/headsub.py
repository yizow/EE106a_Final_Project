#!/usr/bin/env python
import sys
import rospy
from tf2_msgs.msg import TFMessage
import tf
import time
from conf import ingredient_list
shift = False

last_seen = time.time()

import tf

global ar_markers

def listener(tf_topic):
    #sets up listener to the TF topic, which listens to TF Messages, and calls a callback

    print("Initializing node... ")
    rospy.init_node("head_cam")
    print("topic is " + str(tf_topic))

    rospy.Subscriber(tf_topic,TFMessage,callback)
    print("Construction of listener completed")

    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()

def callback(data):
    #run every time i see a TFMessage on tf_topic.
    global last_seen
    global trans
    global rot

    tim = data.transforms[0]
    frame_id = tim.header.frame_id
    child_frame = tim.child_frame_id
    translations = tim.transform.translation
    quaternion = tim.transform.rotation
    tim = tim.header.stamp

    if child_frame in ar_markers and frame_id == "/head_camera":
    print frame_id
        now = rospy.Time.now()
        try:
            print "orig vals"
            print(data)

            ndata = data
            ndata.transforms[0].transform.translation.x = -tftranslations.x
            ndata.transforms[0].transform.translation.y = -translations.y
            ndata.transforms[0].transform.rotation.x = -quaternion.x
            ndata.transforms[0].transform.rotation.y = -quaternion.y
            ndata.transforms[0].child_frame_id = child_frame + "/corrected"
            print "new vals"

            print ndata

        except:
            #print("error with the update \n\r")
            pass

        br = tf.TransformBroadcaster()
        br.sendTransform((ndata.transforms[0].transform.translation.x, ndata.transforms[0].transform.translation.y, ndata.transforms[0].transform.translation.z),
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w), 
        rospy.Time.now(),
        child_frame + "/corrected", "/head_camera")

# Initializes listener for the TF and then closed_loop_pick
def init():
    listener('/tf')

if __name__ == '__main__':
  global ar_markers
  ar_markers = []
  for ingredient in ingredient_list:
    ar_markers.append(ingredient[1])
  print ("Rebroadcasting " + str(ar_markers))
  init()
