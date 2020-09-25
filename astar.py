from Map import Map_Obj # importing the Map_Obj class
from queue import PriorityQueue # import priority queue
import sys # importing sys for command line arguments

def heuristic(posA, posB):
    """
    Calculates the manhattan distance between two points
    :param posA: Position A, the node we are currently on
    :param posB: Position B, the node we want to go to, the goal
    :return: the manhattan distance betwween posA and posB
    """
    # Calculating the manhattan distance
    return abs(posA[0] - posB[0]) + abs(posA[1] - posB[1])

def astar(start, goal, map_obj, moving):
    """
    An implementation of the A* algorithm for finding the shortest
    path from a point to another in the map. Based on Dijkstra's algorithm
    :param start: The start node from where we start the search
    :param goal.: The goal node.
    :param map_obj: The map object which describes the environment
    :param moving: A boolean variable for whether the goal is moving or not
    :return: the manhattan distance betwween posA and posB
    """
    # Extracting the costs for all the connected nodes in the map    
    costs = map_obj.get_maps()[0]
    
    # A dictionary for storing node --> heuristic function
    h = {}
    # Whether a node has been visited before or not
    visited = {}
    # A dictionary for storing the shortest distances to all the nodes in the search tree
    distance = {}
    # Parent node of a node in the search tree
    parent = {}
    """
    Priority queue for storing the (calculated shortest distance, node), so that we 
    always extract the node with the shortest path
    """
    pq = PriorityQueue()

    # Lists can't be hashed, so converting it to a tuple
    start = (start[0], start[1])
    # Calculating the heuristic for the start node
    h[start] = heuristic(start, goal)
    # The start node has no parent
    parent[start] = -1
    # Shortest distance from start to start is 0
    distance[start] = 0

    # Pushing the start node to the priority queue with its heuristic + cost = heuristic since cost is 0
    pq.put((h[start], start))
    # Searching until the priority queue is empty
    while not pq.empty():
        # Extracting the tuple t with the minimum distance
        t = pq.get()
        # Here, f = cost + heuristic, and node is the current node
        f, node = t[0], t[1]
        # Marking the node as visited
        visited[node] = True

        # Task 5
        if moving:
            # Updating the goal
            goal = map_obj.tick()

        # Checking if we have calculated the heuristic for the current node
        if node not in h:
            # Calculating for the current node
            h[node] = heuristic(node, goal)

        """
        If we have not calculated the shortest distance for this node
        we set it to positive infinity since we don't yet know what the
        shortest distance is. If the distance remains infinity after the
        search, it is not in the search tree.
        """
        if node not in distance:
            # Setting distance to node as positive infinity
            distance[node] = float('inf')

        # Checking for duplicates in the priority queue
        if f - h[node] > distance[node]:
            continue
        
        # Exploring all the neighbours of the current node in all 4 directions
        for neighbour in [
            (node[0],node[1]+1),
            (node[0],node[1]-1),
            (node[0]+1,node[1]),
            (node[0]-1,node[1])]:

            # Exploring if the neighbouring node is not an obstacle
            if costs[neighbour[0]][neighbour[1]] != -1:
                # Calculating the cost to the neighbouring node
                weight = costs[neighbour[0]][neighbour[1]]
                # If the neighbouring node has been visited before, there is no point on visiting it again
                if neighbour in visited:
                    continue

                # Calculating the new distance (without the heuristic) for the neighbour 
                newDist = distance[node] + weight

                # Same logic as line 78
                if neighbour not in distance:
                    # Setting the distance to the neighbour as positive infinity
                    distance[neighbour] = float('inf')

                # Relaxation
                if newDist < distance[neighbour]:
                    # Updating the shortest distance
                    distance[neighbour] = newDist
                    # Setting the parent node of the explored neighbour as the node extracted from the priority queue
                    parent[neighbour] = node

                    # Checking if we have calculated the heuristic for the neighbour
                    if neighbour not in h:
                        # Calculating the heuristic for the neigbour node
                        h[neighbour] = heuristic(neighbour, goal)
                    
                    # Pushing the neighbour to the priority queue with the new shortest distance
                    pq.put((newDist + h[neighbour], neighbour))
        
        # If the current node is the goal, we have found the shortest distance to the goal, and we are finished
        if node == goal:
            print(distance[(goal[0], goal[1])])
            return parent, goal
    
    print(distance[(goal[0], goal[1])])
    return parent, goal

def findPath(map_obj, moving):
    """
    Function for drawing the shortest path from start to goal
    :param map_obj: The map object which describes the environment
    :param moving: A boolean variable for whether the goal is moving or not
    :return: None
    """
    # Extracting the start node of the current task
    start = map_obj.get_start_pos()
    # Extracting the goal node of the current task
    goal = map_obj.get_goal_pos()

    # Calling the A* algorithm for getting the parent dictionary and the goal
    parent, goal = astar(start, goal, map_obj, moving)

    # Starting at the goal node to draw the path
    node = (goal[0], goal[1])
    # Continuing until we hit start
    while parent[node] != (start[0], start[1]):
        # Coloring the path
        map_obj.set_cell_value(parent[node], ";")
        # Traversing up the search tree
        node = parent[node]
    
    # Showing the map with the colored path
    map_obj.show_map()

# Main
if __name__ == "__main__":
    # Getting the task number from the command line
    task = int(sys.argv[1])
    # Setting the moving variable as true or false
    moving = task == 5
    # Getting the map for the current task
    map_obj = Map_Obj(task=task)
    # Showing the map with the colored shortest path calculated with the A* algorithm
    findPath(map_obj, moving)