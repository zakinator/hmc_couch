#!/usr/bin/env python
import rospy
import cv
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import colorsys

def callback(data):
  global ledVal
  cv_image = bridge.imgmsg_to_cv(data,"bgr8")
  (reds,greens,blues,x) = cv.Avg(cv_image)

  (h,s,v) = colorsys.rgb_to_hsv(reds,greens,blues)
  s = 1
  (reds,greens,blues) = colorsys.hsv_to_rgb(h,s,v)

  ledVal = 0
  ledVal |= (int(reds) & 0xFF) << 16
  ledVal |= (int(greens) & 0xFF) << 8
  ledVal |= (int(blues) & 0xFF) 

  print int(reds),int(greens),int(blues)

  color = cv.CV_RGB(int(blues),int(greens),int(reds))
  cv.Rectangle(cv_image, (0,0), (100,100), color , thickness = 20)
  cv.ShowImage("image", cv_image)
  cv.WaitKey(1)


ledVal = 0
if __name__ == "__main__":
  global bridge
  global ledVal
  print "starting"
  pub = rospy.Publisher('couchLEDs', Int32)
  rospy.init_node('ambient')
  bridge = CvBridge()
  cv.NamedWindow("image",1)
  image_sub = rospy.Subscriber("/image_raw",Image,callback) 

  print "connected"
  while not rospy.is_shutdown():
    pub.publish(Int32(ledVal))
    rospy.sleep(.001)

  print "stopping"





