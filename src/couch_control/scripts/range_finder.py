#!/usr/bin/python
#############################################
##    Range finder GUI for Neato XV11 laser scanner
##    For use at Harvey Mudd College
##    Authors: Zakkai Davidson
##    Date: 6/13/13
##    How it works: visualize the laser scan data, and the detected lines are labeled green
#############################################
##    NOTE:
##    Run Neato driver first with
##    rosrun neato_mudd driver.py
#############################################
import roslib; roslib.load_manifest('neato_mudd')
import rospy
import cv
import neato_mudd

from std_msgs.msg import *
from sensor_msgs.msg import *
from neato_mudd.msg import *
from neato_mudd.srv import *
from math import *

# class for a generic data holder
class Data:
    def __init__(self): pass    # empty constructor

D = Data()

# window variables
WIN_WIDTH  = 600                # keeps square windows
WIN_HEIGHT = WIN_WIDTH


# line drawing variables
MAX_MAG      = 1000             # unreliable data above 10m
MAG_SCALE    = 100              # 100 pixels per meter
CENTER       = WIN_WIDTH/2
ANGLE_OFFSET = 90               # front of robot faces up on screen
REV          = 360              # 360 degrees per rev (range data is stored in degrees)

# OPTIONS
SHOW_HOUGH = False              # set to true to show Hough transformation image


def rangeGUI():
    """ creates and displays a GUI for the range finder data
        Ranges window: shows range finder values as red lines
        coming from the center of the range finder
        HoughLines window: shows the result of using a Hough
        transformation on image containing the range values as points.
    """
    global D
    
    init_GUI() # initialize images and windows

    D.dangerList = [] # Set up the danger point list
    
    # main loop
    while rospy.is_shutdown() == False:

        # loop through every angle
        for angle in range(REV):
            magnitude = MAG_SCALE*D.ranges[angle]
            x = int(CENTER + magnitude*cos(pi/180*(angle+ANGLE_OFFSET)))  # find x and y coordinates based on the angle
            y = int(CENTER - magnitude*sin(pi/180*(angle+ANGLE_OFFSET)))  # and the length of the line

            # put the danger points into the list
            if x > CENTER - 30 and x < CENTER + 30 and y < CENTER -30 and y > CENTER - 90:
                D.dangerList.append((x,y))
                
            if 0 < magnitude < MAX_MAG: # if data is within "good data" range
                # add line to the ranges image
                cv.Line(D.image, (CENTER,CENTER), (x,y), cv.RGB(255, 0, 0))
                # add dot to image being used in Hough transformation
                cv.Line(D.hough, (x,y), (x,y), cv.RGB(255, 0, 0))

        # wait and check for quit request
        key_code = cv.WaitKey(1) & 255
        key_press = chr(key_code)
        if key_code == 27 or key_press == 'q' : # if ESC or 'q' was pressed
            rospy.signal_shutdown( "Quitting..." )

        # find walls and add to image using Hough transformation
        findHoughLines()

        # show image with range finder data and calculated walls
        cv.ShowImage("Ranges",  D.image)
        
        # show image used in Hough transformation if option is selected
        if SHOW_HOUGH:
            cv.ShowImage("HoughLines", D.color_dst)

        # clear the images for next loop
        cv.Set(D.image, cv.RGB(0, 0, 0))
        cv.Set(D.hough, cv.RGB(0, 0, 0))

        # clear the danger list for next loop
        D.dangerList = []


    D.tank(0, 0) # stops the robot
    print "Quitting..."
            

def init_GUI():
    """ initializes open cv windows and creates images to display """
    global D

    print
    print "Press 'q' in Ranges window to quit"
    
    # create window and image to show range values
    cv.NamedWindow("Ranges")
    cv.MoveWindow("Ranges", WIN_WIDTH/2, WIN_HEIGHT/2)
    D.image = cv.CreateImage((WIN_WIDTH,WIN_HEIGHT), 8, 3) # 8 is pixel depth and 3 is # of channels

    # window for Hough transformation
    if SHOW_HOUGH:
        cv.NamedWindow("HoughLines")
        cv.MoveWindow("HoughLines", WIN_WIDTH/2 + WIN_HEIGHT, WIN_HEIGHT/2)
    # image for Hough transformation
    D.hough = cv.CreateImage((WIN_WIDTH,WIN_HEIGHT), 8, 3) # image used for Hough transformation
    
    # initialize ROS subscription
    rospy.init_node("range_listener", anonymous=True)
    rospy.Subscriber( "laserRangeData", LaserRangeData, range_callback )

    D.laserBroadcaster = rospy.Publisher("scan", LaserScan)

    # initialize ROS service
    rospy.wait_for_service('tank') # wait until the motors are available
    D.tank = rospy.ServiceProxy('tank',Tank)
    
    # give initial values to range data before first callback
    D.ranges =[0]*REV


def range_callback(data):
    """ handles published range data and updates global """
    global D

    D.ranges = data.range_data
    
    # Sending laser data to gmapping
    laserMsg = LaserScan()
    laserMsg.header.stamp = rospy.Time.now()
    laserMsg.header.frame_id = "/base_laser"
    laserMsg.angle_min = pi/2
    laserMsg.angle_max = 2.5*pi
    laserMsg.angle_increment = pi/180.0
    laserMsg.range_min = 0.5
    laserMsg.range_max = 10.0
    laserMsg.ranges = D.ranges
    D.laserBroadcaster.publish(laserMsg)
    print D.ranges    


def findHoughLines():
    """ Uses the Hough transformation to find lines from the sensor
        readings and displays them
    """
    global D

    # initialize lists
    D.lines        = []
    D.theta        = []
    D.distance     = []
    D.midpoint     = []

    # sensitivity options for Hough transformation
    threshold    = 20
    min_line_len = 10
    max_gap_len  = 30
    line_width   = 1
    
    # source for Hough transformation has dots instead of lines
    src = D.hough

    # prepare image and destination for Hough transformation
    dst = cv.CreateImage(cv.GetSize(src), 8, 1)
    D.color_dst = cv.CreateImage(cv.GetSize(src), 8, 3)
    storage = cv.CreateMemStorage(0)
    lines = 0
    cv.Canny(src, dst, 50, 200, 3)
    cv.CvtColor(dst, D.color_dst, cv.CV_GRAY2BGR)

    # apply Hough transformation to find walls
    # For more information, see:
    # http://docs.opencv.org/doc/tutorials/imgproc/imgtrans/hough_lines/hough_lines.html
    lines = cv.HoughLines2(dst, storage, cv.CV_HOUGH_PROBABILISTIC, line_width, \
                           pi / 180, threshold, min_line_len, max_gap_len)

    # draw the danger zone
    cv.Rectangle(D.image, (CENTER - 30,CENTER - 90), (CENTER + 30, CENTER -30), cv.RGB(25,25,112), 2,8)
    
    for line in lines:
        cv.Line(D.color_dst, line[0], line[1], cv.CV_RGB(0, 255, 0), 1, 8)

        # storing the lines and their distances
        D.lines.append((line[0],line[1]))
        x1 = float(line[0][0])
        y1 = float(line[0][1])
        x2 = float(line[1][0])
        y2 = float(line[1][1])
        x3 = float(CENTER)
        y3 = float(CENTER)

        # find the midpoint, the angle, and the distance to center
        midpoint = (int((x1+x2)/2),int((y1+y2)/2))
        theta = atan2((y2-y1),(x2-x1)) / pi * 180

        if (x2 - x1) != 0:
            slope = (y2 - y1) / (x2 - x1)
            intercept = (x2*y1 - x1*y2)/(x2 - x1)
            distance = abs(y3 - slope * x3 - intercept) / sqrt(slope**2 + 1)
        else:
            distance = abs(x2 - x3)
        
        cv.Line(D.image, line[0], line[1], cv.CV_RGB(0, 255, 0), 1, 8)
        cv.Line(D.image, midpoint, midpoint, cv.RGB(255, 255, 255), 4, 8)

        # add data to the list
        D.theta.append(theta)
        D.distance.append(distance)
        D.midpoint.append(midpoint)
    
if __name__ == "__main__":
    rangeGUI()

