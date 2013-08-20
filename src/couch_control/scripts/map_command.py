#!/usr/bin/python
#############################################
## Laser range data handler for the autonomous couch at Harvey Mudd College.
## Adapted from the autonomous hallway navigation for the Neato XV11 from
## Harvey Mudd College.
## Author: Zakkai Davidson
## Date: 8/20/13
#############################################
import roslib; roslib.load_manifest('couch_control')
import rospy
import cv
#import neato_mudd
import time

from std_msgs.msg import *
from couch_control.msg import *
from sensor_msgs.msg import LaserScan
#from neato_mudd.srv import *
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
DANGER_SIZE = 60

# OTHERS
CHECK_TIME = 20
SEND_COUNTER = 20

# Direction
LEFT = "L"
RIGHT = "R"
STRAIGHT = "S"

def rangeGUI():
    """ creates and displays a GUI for the range finder data
        Ranges window: shows range finder values as red lines
        coming from the center of the range finder
        HoughLines window: shows the result of using a Hough
        transformation on image containing the range values as points.
    """
    global D
    
    init_GUI() # initialize images and windows

    # initialize the lists and variables
    D.dangerList = [] # Set up the danger point list
    D.ahead = []
    D.behind = []
    D.left = []
    D.right = []
    D.broadcast = []
    D.send_counter = 0 # counter to turn on the 'send' switch
    D.corner_type = 0 # counter to send the corner type
    D.send = True # at first the send switch turns on
    D.text = "Wait for information" # Text on the display window
    D.waiting = 0 # Counter for waiting map direction
    D.next_dir = ""
    D.default_dir = "Right"
    
    # main loop
    while rospy.is_shutdown() == False:

        # loop through every angle
        for angle in range(REV):
            magnitude = MAG_SCALE*D.ranges[angle]
            x = int(CENTER + magnitude*cos(pi/180*(angle+ANGLE_OFFSET)))  # find x and y coordinates based on the angle
            y = int(CENTER - magnitude*sin(pi/180*(angle+ANGLE_OFFSET)))  # and the length of the line

            # put the danger points into the list
            if x > CENTER - DANGER_SIZE and x < CENTER + DANGER_SIZE and y < CENTER - DANGER_SIZE and y > CENTER - 3*DANGER_SIZE:
                D.dangerList.append((x,y))

            # check the points in the cross zone and put them in four lists
            if x < CENTER - 20 and y > CENTER - 20 and y < CENTER + 10:
                D.left.append((x,y))
            elif x > CENTER + 20 and y > CENTER - 20 and y < CENTER + 10:
                D.right.append((x,y))
            elif y < CENTER - 30 and y > 60 and x < CENTER + 10 and x > CENTER - 10:
                D.ahead.append((x,y))
            elif y > CENTER + 10 and x < CENTER + 10 and x > CENTER - 10:
                D.behind.append((x,y))

            
                
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

        # call wall following algorithm
        wallFollow()

        # show image with range finder data and calculated walls
        cv.ShowImage("Ranges",  D.image)
        
        # show image used in Hough transformation if option is selected
        if SHOW_HOUGH:
            cv.ShowImage("HoughLines", D.color_dst)

        # clear the images for next loop
        cv.Set(D.image, cv.RGB(0, 0, 0))
        cv.Set(D.hough, cv.RGB(0, 0, 0))

        # clear all the lists for next loop
        D.dangerList = []
        D.ahead = []
        D.behind = []
        D.left = []
        D.right = []


    D.tank(0, 0) # stops the robot
    print "Quitting..."
            

def init_GUI():
    """ initializes open cv windows and creates images to display """
    global D

    print
    print "Press 'q' in Ranges window to quit"

    # initialize the font
    D.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1, 1, 0, 2, 8)
    
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

    rospy.Subscriber("scan", LaserScan, range_callback )

    rospy.Subscriber("navCommand", NavCommand, nav_callback )

    # initialize ROS publishers
    D.hallwayType = rospy.Publisher("hallwayType", HallwayType)

    D.motorPub = rospy.Publisher("couchMotors", MotorCommand)

    # initialize ROS service
    #rospy.wait_for_service('tank') # wait until the motors are available
    #D.tank = rospy.ServiceProxy('tank',Tank)

    # replaces the service above
    D.tank = _motor_broadcast
    
    # give initial values to range data before first callback
    D.ranges =[0]*REV


def _motor_broadcast(left, right):
    """ workaround to send motor commands to the couch with minimal changes.
        Function should be referenced as D.tank() to send motor commands in
        the same way as the Neato.
        inputs: motor powers (int) left and right
        output: broadcasts the motor commands to the motorCommand topic
    """
    if left > 80:
        left = 80
    if right > 80:
        right = 80
    # correctly format motor command
    motorPower = MotorCommand()
    motorPower.left  = left
    motorPower.right = right

    # broadcast motor command
    D.motorPub.publish(motorPower)


def range_callback(data):
    """ handles published range data and updates global """
    global D

    D.ranges = data.ranges
    D.ranges = D.ranges[::-1]


def nav_callback(data):
    global D

    D.next_dir = data.command
    print "Received command: " + str(data.command)


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

    # draw the intersection type
    cv.PutText(D.image, D.text, (30,30), D.font, cv.RGB(255,255,255))
    
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
    


###### TODO #######
def wallFollow():
    """ tries to keep the wall lines vertical using arcade control """
    global D
   
    # set up variables and constants
    speed = 40
    k = 1.0
    angle_offset = 15
    distance_offset = 150
    x_sum = 0
    y_sum = 0
    minListNumber = 3

    lineAhead = False
    lineLeft = False
    lineRight = False
    
    
    
    try:
        try:
            # find the average x coordinate of all danger points
            for point in D.dangerList:
                x_sum += point[0]
                y_sum += point[1]
            x_avg = x_sum / len(D.dangerList)
            y_avg = y_sum / len(D.dangerList)
        except:
            pass

        # check all the hough lines and put flags
        for i in range(len(D.midpoint)):
            line_midpoint = D.midpoint[i]
            line_distance = D.distance[i]
            line_theta = D.theta[i]
            line_x = (D.lines[i][0][0],D.lines[i][1][0])
            line_y = (D.lines[i][0][1],D.lines[i][1][1])

            if abs(line_theta) < angle_offset:
                if line_midpoint[1] > CENTER + distance_offset:
                    lineBehind = True
                elif line_midpoint[1] < CENTER and line_midpoint[1] > distance_offset and max(line_x) > CENTER and min(line_x) < CENTER:
                    lineAhead = True
            elif abs(line_theta) >= (90 - angle_offset):
                if line_x[0] < CENTER and line_x[1] < CENTER and max(line_y) < CENTER:
                    lineLeft = True
                elif line_x[0] > CENTER and line_x[1] > CENTER and max(line_y) < CENTER:
                    lineRight = True

        broadcast()

        
        # decide how to tank
        # check if there is any command, if not, do the default thing
        if D.next_dir != "":
            print str(D.next_dir)
            turn(D.next_dir)     
        elif lineAhead:
            if lineRight:
                turn(LEFT)
            else:
                turn(RIGHT)
                
        # danger zone detecting
        elif len(D.dangerList) > minListNumber:
            delta_p = (70 - abs(CENTER - 30 - y_avg))
            if x_avg > CENTER + 5:
                D.tank(100 - delta_p,200 + delta_p)
            elif x_avg < CENTER - 5:
                D.tank(200 + delta_p,100 - delta_p)
        # hallway following
        else:
            theta = 0
            for thetas in D.theta:
                if abs(thetas) > abs(theta):
                    theta = thetas
                    index = D.theta.index(thetas)

            point = (D.midpoint[index][0], D.midpoint[index][1])
            cv.Line(D.image, point, point, cv.CV_RGB(255, 255, 255), 20, 8)           
            delta = 90 - abs(theta)
            if delta < 5:
                delta = 5           
            if theta > 0:
                D.tank(speed - delta, speed + delta)
            else:
                D.tank(speed + delta ,speed - delta)

    except:
        pass


def turn(direction):
    global D
    D.next_dir = ""
    if direction == "L":
        D.text = "Turning Left..."
        D.tank(-100,100)
        rospy.sleep(1.5)
    elif direction == "R":
        D.text = "Turning Right..."
        D.tank(100,-100)
        rospy.sleep(1.5)
    elif direction == "STOP":
        D.tank(0,0)
        rospy.signal_shutdown( "Quitting..." )
    else:
        pass
        
   

    

## broadcast the type of intersection
def broadcast():
    global D

    ahead = False
    behind = False
    left = False
    right = False
    
    if len(D.left) > 5:
        left = True
    if len(D.right) > 5:
        right = True
    if len(D.ahead) > 3:
        ahead = True
    if len(D.behind) > 5:
        behind = True

########## print ahead, left, right
    dir_sum = ahead + left + right

    if dir_sum == 3:
        if D.send:
            D.broadcast.append("D")
    elif dir_sum == 2:
        if (not ahead):         
#######  print "No intersection, move forward"
            D.send_counter += 1
            D.broadcast = []
            D.corner_type = 0            
            if D.send_counter > SEND_COUNTER:
                
                message = HallwayType()   #Broadcast to TMap
              #  message.__setattr__("hall_type", "ST")
                message.hall_type = "ST"
                D.hallwayType.publish(message)
                print "Sent ST"

                D.text = "Going Straight..."
                D.send = True
                D.send_counter = 0
        else:
            if D.send:
                if left:
                    D.broadcast.append("LR")
                elif right:
                    D.broadcast.append("LL")
    elif dir_sum == 1:
        if D.send:
            if ahead:
                D.broadcast.append("TU")
            elif left:
                D.broadcast.append("TR")
            elif right:
                D.broadcast.append("TL")
    else:
        if D.send:
            D.broadcast.append("+")

    if len(D.broadcast) > 1:
        if D.broadcast[-1] == D.broadcast[-2]:
            D.corner_type += 1
        else:
            D.corner_type = 0

    if D.corner_type >= CHECK_TIME:
        send_dir()
        D.tank(0,0)
        rospy.sleep(1)


def send_dir():
    global D
   ## print "Send " + str(D.broadcast[-1])

    message = HallwayType()     #Broadcast to TMap
    message.hall_type = str(D.broadcast[-1])
  #  message.__setattr__("hall_type", str(D.broadcast[-1]))
    D.hallwayType.publish(message) 

    print "Sent corner"

    D.send = False
    D.corner_type = 0
    D.send_counter = 0
    D.broadcast = []


if __name__ == "__main__":
    rangeGUI()
