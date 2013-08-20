#!/usr/bin/python
##########################################################
## Localization and turn by default
## For use with the Neato XV11
## Designed for use at Harvey Mudd College
## Authors: Zakkai Davidson, Jerry Hsiung, Cyrus Huang 
## Date: July 2013
##########################################################
import roslib; roslib.load_manifest('neato_mudd')
import rospy
import cv

from neato_mudd.msg import *
from time import sleep
from neato import xv11
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

# initialize globals in localization
D.START_FOUND = False
D.possibleNodes = []
D.drawPossNodes = []

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
    D.direction = NORTH
    D.commands, D.cmd_path  = D.map.generateCommands(D.start_node, D.dst_node)
    D.commands.append(["STOP","STOP"])
    D.cmd_path = D.cmd_path[1:-1]
    D.node_states[D.start_node] = EMPTY

    # Generate a list of possible nodes for localization
    generateList()
    D.drawPossNodes = map(lambda x: x[0].name, D.possibleNodes)

def generateList():
    """ Generate initial possible node list """
    global D
    # Reset the corner type
    cornerType = ""
    
    # Loop through all nodes
    for node_nums in NODE_NUMS:
        
        # Find the actual reference of the node
        node = D.map.findNode(node_nums)
        if node.type != HALLWAY:
            # Find all the neighbors of the node (Possible direction to come from)
            neighborList = node.getNeighbors()
            
            # Loop through all direction to determine specific type of corner this node is
            for neighbor in neighborList:
                # Get the specific type of corner
                if neighbor != None:
                    neighbor_index = neighborList.index(neighbor)
                    cornerType = getCornerType(node, neighbor_index, neighborList)
                    
                    # Append to the global list
                    D.possibleNodes.append([node,cornerType,DIRECTIONS[neighbor_index]])
      
def getCornerType(node, neighbor_index, neighborList):
    """ Determine/Return the specific corner type of the input 'node'
        Neighbor_Index:
            0: from North
            1: from South
            2: from East
            3: from West.   """
    global D
    cornerType = ""
    direction = DIRECTIONS[neighbor_index] # Direction coming from
    node_type = node.type
    
    # Determine specific T type
    if node_type == T_INT:
        if direction == NORTH:
            if neighborList[2] != None and neighborList[3] != None:
                cornerType = "TU"
            elif neighborList[2] == None:
                cornerType = "TR"
            else:
                cornerType = "TL"
        elif direction == SOUTH:
            if neighborList[2] != None and neighborList[3] != None:
                cornerType = "TU"
            elif neighborList[2] == None:
                cornerType = "TL"
            else:
                cornerType = "TR"
        elif direction == WEST:
            if neighborList[0] != None and neighborList[1] != None:
                cornerType = "TU"
            elif neighborList[0] == None:
                cornerType = "TR"
            else:
                cornerType = "TL"
        else:
            if neighborList[0] != None and neighborList[1] != None:
                cornerType = "TU"
            elif neighborList[0] == None:
                cornerType = "TL"
            else:
                cornerType = "TR"
                
    # Determine specific L type  
    elif node_type == ELBOW:
        if direction == NORTH:
            if neighborList[2] == None:
                cornerType = "LR"
            else:
                cornerType = "LL"
        elif direction == SOUTH:
            if neighborList[2] == None:
                cornerType = "LL"
            else:
                cornerType = "LR"
        elif direction == WEST:
            if neighborList[0] == None:
                cornerType = "LR"
            else:
                cornerType = "LL"
        else:
            if neighborList[0] == None:
                cornerType = "LL"
            else:
                cornerType = "LR"
    
    elif node_type == DEAD_END:
        cornerType = node_type    
    elif node_type == CROSS:
        cornerType = node_type
        
    return cornerType
    
    
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

    if D.START_FOUND:    
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

    else:
        # Localization mode
        if hallway_type != "ST": # if we have met an intersection

            # Localizing:
            filterNodes(hallway_type)
            D.drawPossNodes = map(lambda x: x[0].name, D.possibleNodes)
            if len(D.possibleNodes) == 1:
                print "WE FOUND IT!!!!!: " + str(D.possibleNodes)

                # reset the start node
                resetStart()
                D.START_FOUND = True   # Go into navigation mode

                # send the turn command
                message = NavCommand()
                com_and_type = D.commands.pop() # in format [command, hallway type]
                command       = com_and_type[0]
                message.command = command
                D.commandPub.publish(message)
                
            else:
                # Decide the next move
                estimateNextNodes()
                
                # Turn by default
                message = NavCommand()
                message.command = DE_DIR[hallway_type]
                D.commandPub.publish(message)

    # reset the prev_type
    D.prev_type = hallway_type


def resetStart():
    """reset the start node to the one found"""
    global D

    # reset the previous start_node to be empty
    D.node_states[D.start_node] = EMPTY

    # get the start node
    D.start_node = int(D.possibleNodes[0][0].name)
    D.node_states[D.start_node] = START
    D.curr_node = D.start_node
    
    # adjust the direction
    if D.possibleNodes[0][2] == NORTH:
        D.direction = SOUTH
    elif D.possibleNodes[0][2] == SOUTH:
        D.direction = NORTH
    elif D.possibleNodes[0][2] == EAST:
        D.direction = WEST
    elif D.possibleNodes[0][2] == WEST:
        D.direction = EAST

    # get the start node and set the path!!!
    setPath()

def filterNodes(hallway_type):
    """ filter the nodes based on the intersection type"""
    D.possibleNodes = filter(lambda x: x[1] == hallway_type, D.possibleNodes)
    


def estimateNextNodes():
    """ Determine and estimate the next nodes based on the current possible nodes"""
    # four functions to get neighbor nodes
    def getRightNeighbors(node):
        if node[2] == NORTH:
            return node[0].west, node[0].west.west, EAST
        elif node[2] == SOUTH:
            return node[0].east, node[0].east.east, WEST
        elif node[2] == WEST:
            return node[0].south, node[0].south.south, NORTH
        elif node[2] == EAST:
            return node[0].north, node[0].north.north, SOUTH
    def getLeftNeighbors(node):
        if node[2] == NORTH:
            return node[0].east, node[0].east.east, WEST
        elif node[2] == SOUTH:
            return node[0].west, node[0].west.west, EAST
        elif node[2] == WEST:
            return node[0].north, node[0].north.north, SOUTH
        elif node[2] == EAST:
            return node[0].south, node[0].south.south, NORTH
    def getFrontNeighbors(node):
        if node[2] == NORTH:
            return node[0].south, node[0].south.south, NORTH
        elif node[2] == SOUTH:
            return node[0].north, node[0].north.north, SOUTH
        elif node[2] == WEST:
            return node[0].east, node[0].east.east, WEST
        elif node[2] == EAST:
            return node[0].west, node[0].west.west, EAST
    def getBackNeighbors(node):
        if node[2] == NORTH:
            return node[0].north, node[0].north.north, SOUTH
        elif node[2] == SOUTH:
            return node[0].south, node[0].south.south, NORTH
        elif node[2] == WEST:
            return node[0].west, node[0].west.west, EAST
        elif node[2] == EAST:
            return node[0].east, node[0].east.east, WEST
        
    # check all the node and find their neighbors
    for curr_node in D.possibleNodes:
        if curr_node[1] == "LL":
            """check the left node"""
            nextNeighbor, nextNode, dir_nextNode = getLeftNeighbors(curr_node)
        elif curr_node[1] == "LR" or curr_node[1] == "TU":
            """check the right node"""
            nextNeighbor, nextNode, dir_nextNode = getRightNeighbors(curr_node)
        elif curr_node[1] == "D":
            nextNeighbor, nextNode, dir_nextNode = getBackNeighbors(curr_node)
        else:
            nextNeighbor, nextNode, dir_nextNode = getFrontNeighbors(curr_node)

        # get neighbor list of the "next" node
        next_neighborList = nextNode.getNeighbors()

        # get nextNode cornerType
        neighbor_index = next_neighborList.index(nextNeighbor)
        corner_type_nextNode = getCornerType(nextNode, neighbor_index, next_neighborList)

        # overwrite the current node with the next node
        D.possibleNodes[D.possibleNodes.index(curr_node)] = [nextNode, corner_type_nextNode, dir_nextNode]
    
        
def main():
    """ main loop for displaying UI and processing key presses """
    global NEW_COORDS, D
    
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
            if ((key in D.drawPossNodes) and not D.START_FOUND) or (state == PRESENT):
                cv.Line(D.image, coord, coord, color,thickness=20)
            else:
                cv.Line(D.image, coord, coord, color,thickness=12)
            
            
        # wait and check for quit request
        key_code = cv.WaitKey(10) & 255
        key_press = chr(key_code)
        if key_code == 27 or key_press == 'q' : # if ESC or 'q' was pressed
            rospy.signal_shutdown( "Quitting..." )
        
        
        cv.ShowImage("LibraComplex", D.image)

        cv.Set(D.image, cv.RGB(0, 0, 0))


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
        print "Start node is set as " + str(D.start_node)
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
        print "Dst node is reset, which is " + str(D.dst_node)
    
def setPath():
    """ creates command list from start to dest """
    global D

    for key in D.cmd_path:
        if key != D.dst_node:
            D.node_states[key] = EMPTY

    D.commands, D.cmd_path  = D.map.generateCommands(D.start_node, D.dst_node, D.direction)
    D.commands.append(["STOP","STOP"])

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
