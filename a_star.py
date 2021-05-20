# This class represents a node
next_mov=direction()
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
# Draw a grid


a=[]
def binary_conv(n):
    for i in range(3,-1,-1):
        if(int(n/(2**i))==1):
            n=n-2**i
            a.append("1")   
        else:
            a.append("0") 
    return a    

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
        neighbors = [(x, y+1),(x-1, y), (x+1, y), (x, y-1)]
        neighbors_update = neighbor_update(neighbors,binary_conv(current_value))
        
        # Loop neighbors
        for next in neighbors_update:
            if(next==none):
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
# Check if a neighbor should be added to open list
def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f >= node.f):
            return False
    return True
# The main entry point for this module

def neighbor_update(neighbors,data):
    flag=0
    pattern=binary_conv(data)
    for direction in pattern:
        if(direction==1):
            neighbors.pop(flag)
            neighbors.insert(flag,None)
        flag+=1

    return neighbors

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

def main():
    # Get a map (grid)
    map = {}
    chars = ['c']
    start = (map_detail.current_x, map_detail.current_y)
    end = (map_detail.end_x, map_detail.end_y)
    width = map_detail.width
    height = map_detail.height
    # Open a file
    
    
    # Loop until there is no more lines

   
    path = astar_search(start, end)
    print()
    count=0
    for move in path:
        count+=1
        if count<len(path):
            x=move[0]
            y=move[1]
            x1=(path[count])[0]
            y1=(path[count])[1]
            direc=predict_move(x,y,x1,y1)

            next_mov.direction=direc
            self.direction_publisher.publish(next_mov)
        else:
            pass    
    
    print(path)
    print()
    print('Steps to goal: {0}'.format(len(path)))
    print()
# Tell python to run main method
if __name__ == "__main__": main()