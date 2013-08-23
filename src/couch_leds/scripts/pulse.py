#!/usr/bin/env python
import rospy
import colorsys
from std_msgs.msg import Int32

if __name__ == "__main__":
  print "starting"
  pub = rospy.Publisher('couchLEDs', Int32)
  rospy.init_node('pulser')
  ledVal = 0
  print "connected"
  while not rospy.is_shutdown():
    rospy.sleep(.01)
    print ledVal
    r,g,b = colorsys.hsv_to_rgb(ledVal/255.0,1,255)
    ledVal += 1
    if ledVal == 255:
      ledVal = 0

    val = 0
    val |= (int(r) & 0xFF) << 16
    val |= (int(g) & 0xFF) << 8
    val |= (int(b) & 0xFF) 
    pub.publish(Int32(val))
  print "stopping"
