#!/usr/bin/python

#####################################################
## Globals for use with navigation programs using Tmap
## Designed for use at Harvey Mudd College
## Author: Zakkai Davidson, Jerry Hsiung, Cyrus Huang
## Date: June to July, 2013
#####################################################
import cv

# class for a generic data holder
class Data:
    def __init__(self): pass    # empty constructor

D = Data()

# node states for UI
EMPTY   = 0
PRESENT = 1
VISITED = 2
ON_PATH = 3
START   = 4
DEST    = 5

# colors for node states
COLOR_EMPTY   = cv.RGB(  0,145,255)  # BLUE 
COLOR_PRESENT = cv.RGB(  0,255,  0)  # RED
COLOR_VISITED = cv.RGB( 80, 80, 80)  # GREY
COLOR_ON_PATH = cv.RGB(255,255,  0)  # YELLOW
COLOR_START   = cv.RGB(  0,255,  0)  # RED
COLOR_DEST    = cv.RGB(255,  0,  0)  # GREEN
COLORS = {
    EMPTY: COLOR_EMPTY,
    PRESENT: COLOR_PRESENT,
    VISITED: COLOR_VISITED,
    ON_PATH: COLOR_ON_PATH,
    START: COLOR_START,
    DEST: COLOR_DEST}
    


# Map coords start at S
NODE_COORDS = {
    0: (0,0),
    1: (-25,0),
    2: (-25,30),
    3: (-25,60),
    4: (-12,60),
    5: (0,60),
    6: (0,83),
    7: (0,105),
    8: (50,105),
    9: (100,105),
    10: (100,83),
    11: (100,60),
    12: (50,60),
    13: (120,60),
    14: (120,30),
    15: (120,0),
    16: (92,0),
    17: (110,60),
    18: (-25,-30),
    19: (120,-20),
    20: (25, 60),
    21: (75, 60),
    22: (60,0),
    23: (-25,-60),
    24: (-25,-90),
    25: (-25,-120),
    26: (-25,-150),
    27: (-25,-180),
    28: (-55,-180),
    29: (-85,-180),
    30: (-105,-180),
    31: (-125,-180),
    32: (-125,-125),
    33: (-125,-70),
    34: (-75, -70),
    36: (-35, -70),
    37: (-52.5, -120),
    38: (-80, -120),
    39: (-80, -135),
    40: (-80, -150),
    41: (-82.5, -150),
    42: (-85, -150),
    43: (-85, -165),
    44: (40, -180),
    45: (110, -180),
    46: (110,- 135),
    47: (110,-90),
    48: (100,-60),
    49: (40,-60),
    50: (140,-180),
    51: (170,-180),
    52: (190,-180),
    53: (210,-180),
    54: (210,-120),
    55: (210,-60),
    56: (200,-60),
    57: (190,-60),
    58: (160,-60),
    59: (130,-60),
    60: (120,-10),
    61: (50, 80),
    62: (50, 70),
    63: (-12.5, 0)
}

HALLWAY_LENGTHS = {
    0: 25,
    2: 60,
    4: 25,
    6: 45,
    8: 100,
    10: 45,
    14: 60,
    16: 55,
    17: 20,
    18: 60,
    49: 125,
    24: 60,
    26: 60,
    28: 60,
    30: 40,
    32: 110,
    34: 100,
    37: 55,
    39: 60,
    41: 5,
    43: 60,
    44: 135,
    46: 90,
    50: 60,
    52: 40,
    54: 120,
    56: 20,
    58: 60,
    60: 20,
    20: 50,
    21: 50,
    62: 20,
    63: 25
          }

# initial states of nodes
D.node_states = {
    0: START,
    1: 0,
    2: 0,
    3: 0,
    4: 0,
    5: 0,
    6: 0,
    7: 0,
    8: 0,
    9: 0,
    10: 0,
    11: 0,
    12: 0,
    13: 0,
    14: 0,
    15: 0,
    16: 0,
    17: 0,
    18: 0,
    19: 0,
    20: 0,
    21: 0,
    22: DEST,
    23: 0,
    24: 0,
    25: 0,
    26: 0,
    27: 0,
    28: 0,
    29: 0,
    30: 0,
    31: 0,
    32: 0,
    33: 0,
    34: 0,
    36: 0,
    37: 0,
    38: 0,
    39: 0,
    40: 0,
    41: 0,
    42: 0,
    43: 0,
    44: 0,
    45: 0,
    46: 0,
    47: 0,
    48: 0,
    49: 0,
    50: 0,
    51: 0,
    52: 0,
    53: 0,
    54: 0,
    55: 0,
    56: 0,
    57: 0,
    58: 0,
    59: 0,
    60: 0,
    61: 0,
    62: 0,
    63: 0}

# helpful key list
NODE_NUMS = NODE_COORDS.keys()

# openCV window constants
WIN_HEIGHT = 500
WIN_WIDTH  = 500

# node types
DEAD_END = "D"
HALLWAY  = "H"
ELBOW    = "L"
T_INT    = "T"
CROSS    = "+"

# directions
NORTH = "N"
SOUTH = "S"
EAST  = "E"
WEST  = "W"
DIRECTIONS = [NORTH, SOUTH, EAST, WEST]

# turns
LEFT     = "L"
RIGHT    = "R"
STRAIGHT = "S"
TURNAROUND = "TRND"
TURN = [LEFT,RIGHT,STRAIGHT,TURNAROUND]

L_PAIRS  = [[SOUTH,EAST],[NORTH,WEST],[EAST,NORTH],[WEST,SOUTH]] # order: [current dir, next dir]
R_PAIRS  = [[NORTH,EAST],[SOUTH,WEST],[EAST,SOUTH],[WEST,NORTH]]
S_PAIRS  = [[NORTH,NORTH],[SOUTH,SOUTH],[EAST,EAST],[WEST,WEST]]
B_PAIRS  = [[NORTH,SOUTH],[SOUTH,NORTH],[EAST,WEST],[WEST,EAST]]

# default direction during localization
DE_DIR = {
    "TU": RIGHT,
    "TR": STRAIGHT,
    "TL": STRAIGHT,
    "LL": LEFT,
    "LR": RIGHT,
    "D": TURNAROUND
    }

# maps
LIBRA = [
    [ 0,63, WEST],
    [ 1, 2,NORTH],
    [ 2, 3,NORTH],
    [ 3, 4, EAST],
    [ 4, 5, EAST],
    [ 5, 6,NORTH],
    [ 5,20, EAST],
    [ 6, 7,NORTH],
    [ 7, 8, EAST],
    [ 8, 9, EAST],
    [ 9,10,SOUTH],
    [20,12, EAST],
    [12,21, EAST],
    [11,21, WEST],
    [10,11,SOUTH],
    [11,17, EAST],
    [17,13, EAST],
    [13,14,SOUTH],
    [14,15,SOUTH],
    [15,16, WEST],
    [16,22, WEST],
    [ 1,18,SOUTH],
    [18,23,SOUTH],
    [23,49, EAST],
    [49,48, EAST],
    [23,24,SOUTH],
    [24,25,SOUTH],
    [25,26,SOUTH],
    [26,27,SOUTH],
    [27,28, WEST],
    [28,29, WEST],
    [29,30, WEST],
    [30,31, WEST],
    [31,32,NORTH],
    [32,33,NORTH],
    [33,34, EAST],
    [34,36, EAST],
    [25,37, WEST],
    [37,38, WEST],
    [38,39,SOUTH],
    [39,40,SOUTH],
    [40,41, WEST],
    [41,42, WEST],
    [42,43,SOUTH],
    [43,29,SOUTH],
    [27,44, EAST],
    [44,45, EAST],
    [45,46,NORTH],
    [46,47,NORTH],
    [45,50, EAST],
    [50,51, EAST],
    [51,52, EAST],
    [52,53, EAST],
    [53,54,NORTH],
    [54,55,NORTH],
    [55,56, WEST],
    [56,57, WEST],
    [57,58, WEST],
    [58,59, WEST],
    [15,60,SOUTH],
    [60,19,SOUTH],
    [12,62,NORTH],
    [62,61,NORTH],
    [63, 1, WEST]]

