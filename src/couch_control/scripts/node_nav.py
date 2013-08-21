#!/usr/bin/python

#######################################
## Node based hallway navigation of the Libra Complex for the autonomous couch
## at Harvey Mudd College. Adapted from the hallawy navigation program for the
## Neato XV11.
## Author: Zakkai Davidson
## Date: 8/20/13
#######################################

import roslib; roslib.load_manifest('couch_control')
import rospy
import cv

from couch_control.msg import *
from time import sleep
#from neato import xv11
from math import *
from navigation import *

# image scaling globals to be set in calibrateCoords()
COORD_WIDTH  = 0
COORD_HEIGHT = 0
WIDTH_SCALE  = 0
HEGIHT_SCALE = 0
MARGIN       = 10
X_OFFSET     = 0
Y_OFFSET     = 0

NEW_COORDS   = {} # dict for new tranformed coordinates

def initUI():
    """ initializes the UI and ROS publishers/subscribers """
    global D
    
    # initialize openCV window
    cv.NamedWindow("LibraComplex")
    cv.MoveWindow("Ranges", WIN_WIDTH/2, WIN_HEIGHT/2)
    D.image = cv.CreateImage((WIN_WIDTH,WIN_HEIGHT), 8, 3) # 8 is pixel depth and 3 is # of channels

    # initialize mouse click callback
    # see http://docs.opencv.org/modules/highgui/doc/user_interface.html#setmousecallback
    cv.SetMouseCallback("LibraComplex", onMouse, None)
    
    # initialize ROS publishers and subscribers
    rospy.init_node("node_nav", anonymous=True)
    rospy.Subscriber("hallwayType", HallwayType, command_callback)
    D.commandPub = rospy.Publisher("navCommand", NavCommand)

    # prepare map with TMap
    D.prev_type = None
    D.map       = TMap()
    D.map.generateMap(LIBRA)
    D.start_node = 0
    D.dst_node   = 22
    D.curr_node  = D.start_node
    D.commands, D.cmd_path  = D.map.generateCommands(D.start_node, D.dst_node)
    D.commands.append(["STOP","STOP"])
    D.cmd_path = D.cmd_path[1:-1]
    
    
    
def coordTransform( node ):
    """ translates map coordinates into pixel coordinates """
    global NEW_COORDSs
    
    node_x = node[0]
    node_y = node[1]

    new_x = int(WIDTH_SCALE*(node_x + X_OFFSET + MARGIN))   # position + offset + margin
    new_y = int(WIN_HEIGHT - HEIGHT_SCALE*(node_y + Y_OFFSET + MARGIN)) # flip y-axis
    
    return (new_x, new_y)

def calibrateCoords():
    """ calibrates coordinate transformation to fit given map coordinates """
    global D, COORD_WIDTH, COORD_HEIGHT, WIDTH_SCALE, HEIGHT_SCALE, X_OFFSET, Y_OFFSET

    coords = NODE_COORDS.values()

    # isolate x and y values to use for calibration
    x_coords = [ point[0] for point in coords ]
    y_coords = [ point[1] for point in coords ]
    
    min_x  = min(x_coords)
    max_x  = max(x_coords)
    width  = max_x - min_x
    COORD_WIDTH = width
    
    min_y  = min(y_coords)
    max_y  = max(y_coords)
    height = max_y - min_y
    COORD_HEIGHT = height

    X_OFFSET     = abs(min_x)   # offset prevents negative pixel values
    Y_OFFSET     = abs(min_y)
    WIDTH_SCALE  = (WIN_WIDTH)/float(COORD_WIDTH + 2*MARGIN)    # ratio of maximum lengths for scaling
    HEIGHT_SCALE = (WIN_HEIGHT)/float(COORD_HEIGHT + 2*MARGIN)  # 2*MARGIN is for margin on boths sides
    

def command_callback(data):
    """ reaction to receiving hallway type broadcast
        responds with direction command
    """
    global D
    
    hallway_type = data.hall_type
    
    if hallway_type != D.prev_type: # if we have moved from corner to hallway or vice versa
        message = NavCommand()
        com_and_type = D.commands.pop() # in format [command, hallway type]
        command       = com_and_type[0]
        expected_type = [com_and_type[1]]

        # adjustment of intersection types for communicating with publisher
        if expected_type == [T_INT]:
            expected_type = ["TR", "TL", "TU"]
        elif expected_type == [HALLWAY]:
            expected_type = ["ST"]
        elif expected_type == ["STOP"]:
            expected_type = ["TR", "TL", "TU","LR", "LL","ST","D"]
        else:
            expected_type = ["LR", "LL"]
                
        if hallway_type in expected_type:
            message.__setattr__("command", command)
            D.commandPub.publish(message)
            
            # update colors as robot travels
            D.node_states[D.curr_node] = VISITED
            D.curr_node = D.cmd_path.pop()
            D.node_states[D.curr_node] = PRESENT
        else:
            D.commands.append(com_and_type)

    D.prev_type = hallway_type
        
def main():
    """ main loop for displaying UI and processing key presses """
    global NEW_COORDS
    
    calibrateCoords() # calibrate scaling based on map coordinates
    
    while rospy.is_shutdown() == False:
        for key in NODE_NUMS:
            coord = NODE_COORDS[key]
            transf_coord = coordTransform(coord)
            NEW_COORDS[key] = transf_coord

        # draw hallway lines
        for edge in LIBRA:
            src_node = edge[0]
            dst_node = edge[1]
            src = NEW_COORDS[src_node]
            dst = NEW_COORDS[dst_node]
            cv.Line(D.image, src, dst, cv.RGB(  0,145,255),thickness=2)

        # draw node circles
        for key in NODE_NUMS:
            coord = NEW_COORDS[key]
            state = D.node_states[key]
            color = COLORS[state]
            cv.Line(D.image, coord, coord, color,thickness=12)
            
            
        # wait and check for quit request
        key_code = cv.WaitKey(10) & 255
        key_press = chr(key_code)
        if key_code == 27 or key_press == 'q' : # if ESC or 'q' was pressed
            rospy.signal_shutdown( "Quitting..." )
        
        
        cv.ShowImage("LibraComplex", D.image)


def onMouse(event, x, y, flags, param):
    """ the method called when the mouse is clicked """
    global D

    # clicked the left button
    if event==cv.CV_EVENT_LBUTTONDOWN:
        # reset old point's state
        D.node_states[D.start_node] = EMPTY
        
        closest_node = 0
        min_dist     = WIN_WIDTH # initial minimum with large value

        # clicked node is closest to click
        for key in NODE_NUMS:
            node_coord = NEW_COORDS[key]
            node_x = node_coord[0]
            node_y = node_coord[1]

            dist = sqrt((x - node_x)**2 + (y - node_y)**2)
            
            if dist < min_dist:
                closest_node = key
                min_dist = dist

        D.node_states[closest_node] = START
        D.start_node = closest_node
        D.curr_node  = D.start_node
        
        
    # clicked the right button
    if event==cv.CV_EVENT_RBUTTONDOWN:
        # reset old point's state
        D.node_states[D.dst_node] = EMPTY
        
        closest_node = 0
        min_dist     = WIN_WIDTH # initial minimum with large value

        # clicked node is closest to click
        for key in NODE_NUMS:
            node_coord = NEW_COORDS[key]
            node_x = node_coord[0]
            node_y = node_coord[1]

            dist = sqrt((x - node_x)**2 + (y - node_y)**2)
            
            if dist < min_dist:
                closest_node = key
                min_dist = dist

        D.node_states[closest_node] = DEST
        D.dst_node = closest_node
    
        setPath() # creates command list to clicked point
        
def setPath():
    """ creates command list from start to dest """
    for key in D.cmd_path:
        if key != D.dst_node:
            D.node_states[key] = EMPTY
    
    D.commands, D.cmd_path  = D.map.generateCommands(D.start_node, D.dst_node)
    D.commands.append(["STOP","STOP"])
    D.commands.reverse()
    D.commands.pop() # ignore first command
    D.commands.reverse()

    D.cmd_path = D.cmd_path[1:-1] # keep endpoints green and red
    print D.commands
    print D.cmd_path

    
    
    for key in D.cmd_path:
        D.node_states[key] = ON_PATH
    D.commands.reverse()    # reverse commands to prepare for pop()
    D.cmd_path.reverse()
    
if __name__ == "__main__":
    initUI()
    main()
