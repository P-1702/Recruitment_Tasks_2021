#!/usr/bin/env python3

import sys
import rospy

from path_planning.msg import direction, map_detail
from MapClass import Map

class Node:
            # Initialize the class
            def __init__(self, position:(), parent:()):
                self.position = position
                self.parent = parent
                self.g = 0 # Distance to start node
                self.h = 0 # Distance to goal node
                self.f = 0 # Total cost
            # Compare nodes
            def __eq__(self, other):
                return self.position == other.position
            # Sort nodes
            def __lt__(self, other):
                 return self.f < other.f
            # Print node
            def __repr__(self):
                return ('({0},{1})'.format(self.position, self.f))

class PlannerNode:
    def __init__(self):
        self.direction_publisher = rospy.Publisher("/direction", direction, queue_size=10)
        # This is the publisher which will publish the direction for the bot to move
        # A general format for publishing has been given below

        self.walls_subscriber = rospy.Subscriber("/walls", map_detail, self.wall_callback)
        # This is the subscriber that will listen for the details about the map that the bot will aquire
        # This data will be send to the wall_callback function where it should be handled

        rospy.sleep(5) # a delay of some time to let the setup of the subscriber and publisher be completed

        # Since we know that the first step the bot will take will be down, we can simply do it here
        temp_val = direction() # make an object of the message type of the publisher
        temp_val.direction = 'down' # assign value to the object. Refer the custom direction.msg in the msg directory
        self.direction_publisher.publish(temp_val) # publish the object

    def wall_callback(self, map_detail):
        # this function will be called everytime the map sends data regarding the map on the '/walls' topic
        # you will recieve the data in the form of the map_detail variable which is an object of custom message type map_detail.msg from the msg directory
        print(map_detail)
      
        def binary_conv(n):
            a=[]
            for i in range(3,-1,-1):
                if(int(n/(2**i))==1):
                    n=n-2**i
                    a.append("1")   
                else:
                    a.append("0") 
            return a

        def neighbor_update(x,y,data):
            neighbors = [(x, y+1),(x-1, y), (x+1, y), (x, y-1)]
            flag=0
            pattern=binary_conv(data)
            for direction in pattern:
                if(direction==1):
                    neighbors.pop(flag)
                    neighbors.insert(flag,None)
                flag+=1

            return neighbors

        # Check if a neighbor should be added to open list
        def add_to_open(open, neighbor):
            for node in open:
                if (neighbor == node and neighbor.f >= node.f):
                    return False
            return True
        # The main entry point for this module
            
        def predict_move(x,y,x1,y1):
            if(x==x1 and y1-y==1):
                value="up"
            if(x==x1 and y1-y==-1):
                value="down"
            if(x1-x==1 and y1==y):
                value="right"
            if(x1-x==-1 and y1==y):
                value="left"
            return value


       # A* search
        def astar_search(start, end):

            # Create lists for open nodes and closed nodes
            open = []
            closed = []
            # Create a start node and an goal node
            start_node = Node(start, None)
            goal_node = Node(end, None)
            # Add the start node
            open.append(start_node)

            # Loop until the open list is empty
            while len(open) > 0:
                # Sort the open list to get the node with the lowest cost first
                open.sort()
                # Get the node with the lowest cost
                current_node = open.pop(0)
                current_value= map_detail.current_value
                # Add the current node to the closed list
                closed.append(current_node)

                # Check if we have reached the goal, return the path
                if current_node == goal_node:
                    path = []
                    while current_node != start_node:
                        path.append(current_node.position)
                        current_node = current_node.parent
                    #path.append(start) 
                    # Return reversed path

                    return path[::-1]
                # Unzip the current node position
                (x, y) = current_node.position
                # Get neighbors
                neighbors_new = neighbor_update(x,y,current_value)

                # Loop neighbors
                for next in neighbors_update:
                    if(next==None):
                        continue
                    # Get value from map
                    # Create a neighbor node
                    neighbor = Node(next, current_node)
                    # Check if the neighbor is in the closed list
                    if(neighbor in closed):
                        continue
                    # Generate heuristics (Manhattan distance)
                    neighbor.g = abs(neighbor.position[0] - start_node.position[0]) + abs(neighbor.position[1] - start_node.position[1])
                    neighbor.h = abs(neighbor.position[0] - goal_node.position[0]) + abs(neighbor.position[1] - goal_node.position[1])
                    neighbor.f = neighbor.g + neighbor.h
                    # Check if neighbor is in open list and if it has a lower f value
                    if(add_to_open(open, neighbor) == True):
                        # Everything is green, add neighbor to open list
                        open.append(neighbor)
            # Return None, no path is found
            return None


        start = (map_detail.current_x, map_detail.current_y)
        end = (map_detail.end_x, map_detail.end_y)
        width = map_detail.width
        height = map_detail.height

        path = astar_search(start, end)
        count=0
        for move in path:
            count+=1
            if count<len(path):
                        x=move[0]
                        y=move[1]
                        x1=(path[count])[0]
                        y1=(path[count])[1]
                        direc=predict_move(x,y,x1,y1)
                        next_mov = direction()
                        next_mov.direction=direc
                        self.direction_publisher.publish(next_mov)
           else:
                        pass    
        print('Steps to goal: {0}'.format(len(path)))

    # This class represents a node
        


        


if __name__ == '__main__':
    rospy.init_node('planner_node')
    PlannerNode()
    rospy.spin()
        

        
