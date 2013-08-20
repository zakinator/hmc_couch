#!/usr/bin/python
##########################################################
## Topographical map navigator for use with the Neato XV11
## Designed for use at Harvey Mudd College
## Authors: Zakkai Davidson
## Date: 6/17/13
##########################################################
from collections import deque
from globals import *

class TMap:
    """ main map class """
    def __init__(self):
        self.start = TMapNode(0, DEAD_END)

    def __repr__(self):
        """ prints the edge representation of the map """
        edge_list = self.generateEdges()
        return str(edge_list)

    def findNode(self, node_to_find):
        """ finds selected node in the map and returns it
            input: a string node to find
            output: the node to find, false if not in the map
        """
        nodes_stack = []    # stack for looping through all edges


        # put the start on the stack
        nodes_stack.append(self.start)
        
        # loop through all nodes
        while len(nodes_stack) > 0:
                
            node = nodes_stack.pop() # take node off the top of the stack
            if node.name == node_to_find: # compare names to see if node is found
                self.resetNodes()
                return node
            node.visited = True
            neighbors_list = node.getNeighbors() # get the neighbors of that node    
        
            # loop through each neighbor
            for neighbor in neighbors_list:
                if neighbor != None and neighbor.visited == False: # if there is a neighbor,
                    nodes_stack.append(neighbor) # if not, add the node to the stack

        # reset visited flags
        self.resetNodes()
                    
        # if none of the nodes are named node, then node is not in the map
        return False

    
    def generateEdges(self):
        """ generates an edge list that contains all edges of the graph """
        edges = []          # edge list to return
        nodes_stack = []    # stack for looping through all edges

        # put the start on the stack
        nodes_stack.append(self.start)

        # loop through all nodes
        while len(nodes_stack) > 0:
            node = nodes_stack.pop() # take node off the top of the stack
            node.visited = True
            neighbors_list = node.getNeighbors() # get the neighbors of that node

            # loop through each neighbor
            for neighbor in neighbors_list:
                # if there is a neighbor and we haven't been there,
                if neighbor != None and neighbor.visited == False:
                    index  = neighbors_list.index(neighbor)
                    direct = DIRECTIONS[index]
                    edge = [node.name, neighbor.name, direct]
                    edges.append(edge) # add the edge to the list
                    nodes_stack.append(neighbor) # and the node to the stack

        # reset visited flags
        self.resetNodes()
        
        return edges

    def resetNodes(self):
        """ resets all nodes to not_visited """
        nodes_stack = []    # stack for looping through all edges

        # put the start on the stack
        nodes_stack.append(self.start)
        
        # loop through all nodes
        while len(nodes_stack) > 0:
            node = nodes_stack.pop() # take node off the top of the stack
            node.visited = False     # reset visited flag
            node.parent  = None
            neighbors_list = node.getNeighbors() # get the neighbors of that node

            
            # loop through each neighbor
            for neighbor in neighbors_list:
                if neighbor != None and neighbor.visited == True: # if we have been to the neighbor
                    nodes_stack.append(neighbor) # then we need to reset it too
    
    def generateMap(self, all_edges):
        """ generates a map from a list of edges, starting at S
            input: a list of edges in the format [src, dst, direct]
                   e.g. [1,2,N] would be a Northern connection from 1 to 2
            NOTE: every edges only needs to be one directional. The map will
                  make all nodes connect in both directions automatically
        """
        nodes_stack = []

        # put the start on the stack
        
        nodes_stack.append(self.start)
        # loop until out of edges
        while len(all_edges) > 0:
            src_node = nodes_stack.pop()
            src  = src_node.name # src is the name of node node
            # find all edges that have src as their start
            src_edges = filter(lambda(edge): edge[0]==src, all_edges)
            # modify edge list to no longer contain those edges
            all_edges = filter(lambda(edge): edge[0]!=src, all_edges)

            # loop through selected edges
            for edge in src_edges:
                dst    = edge[1] # dst is the name of a node
                direct = edge[2]
                dst_node = self.findNode(dst) # checks for the node dst
                
                if dst_node == False: # check if dst node doesn't exist
                    dst_node = TMapNode(dst, DEAD_END) # if it isn't there, create it

                # add dst to the stack to be checked later if therea are more edges
                nodes_stack.append(dst_node)
                
                if direct == NORTH:
                    src_node.setNorth(dst_node)
                elif direct == SOUTH:
                    src_node.setSouth(dst_node)
                elif direct == EAST:
                    src_node.setEast(dst_node)
                elif direct == WEST:
                    src_node.setWest(dst_node)

        
                

    def mapBFS(self, start, end):
        """ returns the shortest path from start to dest
            inputs: (optional) two strings start and dest that are names of nodes
                    default path is from hot air lab to robot lab
            For information on deque, see
            http://docs.python.org/2/library/collections.html#collections.deque
        """
        nodes_to_check = deque()    # create the queue for BFS
        path           = []         # list to contain nodes on path

        # enqueue the first node and change self.start reference

        start = self.findNode(start)
        self.start = start # useful for resetting nodes
        nodes_to_check.appendleft(start)

        # loop over all nodes
        while len(nodes_to_check) > 0:
            
            current = nodes_to_check.pop()
            current.visited = True

            # if we have found the end, then return the path
            if current.name == end:
                path = []
                while current.parent != None:
                    path.append(current)
                    current = current.parent
                path.append(start) # include starting node in path
                path.reverse()          # reverse list to put nodes in current order
                self.resetNodes()       # reset visited flags
                return path

            # if we haven't found the end, then keep searching
            neighbors_list = current.getNeighbors()
            
            # loop through each neighbor
            for neighbor in neighbors_list:
                if neighbor != None and neighbor.visited == False: # if there is a neighbor,
                    nodes_to_check.appendleft(neighbor) # if not, add the node to the stack
                    neighbor.visited = True
                    neighbor.parent  = current

        # reset visited flags
        self.resetNodes()
        
        # if no path is found, return False
        return False


    def generateCommands(self, start, end, curr_dir = NORTH):
        """ takes a path as generated in TMap.mapBFS() and outputs
            a list of commands in string format
            input: a list of TMapNodes as outputted by TMap.mapBFS()
            output: a list of strings to be parsed as commands
        """
        # get shortest path from BFS
        path = self.mapBFS(start,end)
        
        # for returning
        name_path = [ node.name for node in path ]
        
        path.reverse()      # puts first task at end in preparation for pop()
        current   = path.pop()
        curr_type = current.type
        commands  = []
        # loop through the whole path
        while len(path) > 0:
            
            next_node = path.pop()
            neighbors_list = current.getNeighbors()
            dir_index = neighbors_list.index(next_node) # find the index of the next node
            next_dir  = DIRECTIONS[dir_index]           # figure out which direction the next node is in
            dir_pair  = [curr_dir, next_dir]
            
            # compare directions to decide command
            if dir_pair in L_PAIRS:
                commands.append([LEFT,curr_type])
            elif dir_pair in R_PAIRS:
                commands.append([RIGHT,curr_type])
            elif dir_pair in S_PAIRS:
                commands.append([STRAIGHT,curr_type])
            elif dir_pair in B_PAIRS:
                commands.append([TURNAROUND,curr_type])
            else:
                commands.append(["Error",curr_type])

            # set next to current for next loop
            current   = next_node
            curr_dir  = next_dir
            curr_type = current.type

        return commands, name_path

class TMapNode:
        """ individual node class for TMap """
        def __init__(self, name, node_type, north=None, south=None, east=None, west=None):
            self.name    = name
            self.type    = node_type
            self.north   = north
            self.south   = south
            self.east    = east
            self.west    = west
            self.visited = False
            self.parent  = None
            self.present = False

        def __repr__(self):
            """ returns the name and type of the node """
            str_to_print = "Name: " + str(self.name) + ", Type: " + str(self.type)
            return str_to_print
        
        def getNeighbors(self):
            """ returns all neighbors in an array in the order NSEW """
            return [self.north, self.south, self.east, self.west]

        def getNorth(self):
            """ returns the neighbor to the North """
            return self.north

        def getSouth(self):
            """ returns the neighbor to the South """
            return self.south

        def getEast(self):
            """ returns the neighbor to the East """
            return self.east

        def getWest(self):
            """ returns the neighbor to the West """
            return self.west

        def getType(self):
            """ returns the type of the node """
            return self.type

        def setNorth(self, node):
            """ sets the neighbor to the North to given node """
            self.north = node
            node.south = self
            self.update()
            node.update()

        def setSouth(self, node):
            """ sets the neighbor to the South to given node """
            self.south = node
            node.north = self
            self.update()
            node.update()

        def setEast(self, node):
            """ sets the neighbor to the East to given node """
            self.east = node
            node.west = self
            self.update()
            node.update()

        def setWest(self, node):
            """ sets the neighbor to the West to given node """
            self.west = node
            node.east = self
            self.update()
            node.update()

        def setType(self, node_type):
            """ sets the node to input node_type """
            self.type = node_type

        def update(self):
            """ updates the node to the correct type """
            neighbors = self.getNeighbors()
            neighbors = filter(None, neighbors)
            num = len(neighbors) # find the number of neighbors to determine edge type
            # update the type depending on the number of neighbors
            if num == 1:
                self.type = DEAD_END
            elif num == 2:
                # check if hallway or corner
                if (self.north != None and self.south != None) or (self.east != None and self.west != None):
                    self.type = HALLWAY
                else:
                    self.type = ELBOW
            elif num == 3:
                self.type = T_INT
            elif num == 4:
                self.type = CROSS
