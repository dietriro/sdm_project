import numpy as np
from Queue import PriorityQueue
from Node import Node
from math import floor, atan2

class ARCThetaStar:
    '''
    A Class for the calculation of the shortest path using the A-Star Algorithm
    '''
    # The queue containing all nodes that have not been visited so far
    open = None
    # The queue containing all visited nodes
    closed = None
    # The map for the path planning algorithm
    map = None
    # The epsilon value for calculating the heuristic value
    epsilon = None
    # The maximum rate change of the orientation for the robot
    r_max = None
    # The velocity of the robot
    v = None
    # The possible actions within the map
    actions = np.array([[ 0,  1],   # North
                        [ 1,  0],   # East
                        [ 0, -1],   # South
                        [-1,  0]])  # West

    def __init__(self, map, open_size=50000, closed_size=50000, epsilon=1.0,
                 r_max=1.0, v=2.0):
        '''
        Constructor of ARCThetaStar class.
        :param map: The map that the planner is going to use as a numpy array with values between 0 and 1, where 0 means the tile is free and 1 the tile is occupied.
        :param open_size: The maximum size of the open queue.
        :param closed_size: The maximum size of the closed queue.
        :param epsilon: The epsilon value used for calculating the heuristic value.
        '''
        self.map = map
        self.open = PriorityQueue(open_size)
        self.closed = PriorityQueue(closed_size)
        self.epsilon = epsilon
        self.r_max = r_max
        self.v = v

    def line_of_sight(self, id_start, id_goal):
        '''
        Checks the direct path between two nodes for occupancy.
        :param id_start: The id of the start node.
        :param id_end: The id of the goal node.
        :return: True, if the path is not occupied, False otherwise.
        '''
        pos_s = self.pos_from_id(id_start)
        pos_g = self.pos_from_id(id_goal)
        
        xs = pos_s[0]
        ys = pos_s[1]
        xe = pos_g[0]
        ye = pos_g[1]
        
        dx = xe - xs
        dy = ye - ys
        
        f = 0
        
        if dy < 0:
            dy = -dy
            sy = -1
        else:
            sy = 1
            
        if dx < 0:
            dx = -dx
            sx = -1
        else:
            sx = 1
            
        if dx >= dy:
            while not xs == xe:
                f += dy
                if f >= dx:
                    if self.map[int(xs+((sx-1)/2)), int(ys+((sy-1)/2))]:
                        return False
                    ys += sy
                    f -= dx
                if not f == 0 and self.map[int(xs+((sx-1)/2)), int(ys+((sy-1)/2))]:
                    return False
                if dy == 0 and self.map[int(xs+((sx-1)/2)), int(ys)] and \
                        self.map[int(xs+((sx-1)/2)), int(ys-1)]:
                    return False
                xs += sx
        else:
            while not ys == ye:
                f += dx
                if f >= dy:
                    if self.map[int(xs+((sx-1)/2)), int(ys+((sy-1)/2))]:
                        return False
                    xs += sx
                    f -= dy
                if not f == 0 and self.map[int(xs+((sx-1)/2)), int(ys+((sy-1)/2))]:
                    return False
                if dy == 0 and self.map[int(xs), int(int(ys+((sy-1)/2)))] and \
                        self.map[int(xs-1), int(ys+((sy-1)/2))]:
                    return False
                ys += sy
        return True
        
    def arc_check_angle(self, n_cur, n_next):
        # o_cur = atan((n_next.pos[1]-n_cur.pos[1]) / (n_next.pos[0]-n_cur.pos[0]))
        o_cur = atan2((n_next.pos[1]-n_cur.pos[1]), (n_next.pos[0]-n_cur.pos[0]))
        
        d = self.h_euc(n_next.pos, n_cur.pos)
        
        alpha = o_cur-n_cur.o
        
        # print(alpha)
        
        # Check if angular rate change is within the max value
        if alpha*self.v/d > self.r_max:
            # print('FALSE')
            return False
        
        # Update new angular change
        n_next.o = alpha
        
        # print('TRUE')
        return True
        
        
    def get_node(self, index):
        '''
        Checks whether a Node with the given id is listed in the queue.
        :param queue: The queue that is searched.
        :param index: The id of the Node that is searched for.
        :return: Either True or False, depending on the success.
        '''
        for n in self.open.queue:
            if n.id == index:
                return n
        for n in self.closed.queue:
            if n.id == index:
                return n
        return None
    
    def update_node(self, n_new):
        '''
        Updates the g and parent value of a node within the open queue, in case the g value is smaller than the old one.
        :param n_new: The node with the new values.
        :return: True, if the node was found, false if not.
        '''
        if not self.arc_check_angle(self.get_node(n_new.parent), n_new):
            return
        for n in self.open.queue:
            if n.id == n_new.id:
                if n_new.g <= n.g:
                    n.g = n_new.g
                    n.parent = n_new.parent
                    n.o = n_new.o
                    return True
        return False
            
    def cost(self, id_start, id_goal):
        '''
        Calculates the cost between two nodes based on the euclidean distance between them.
        :param id_start: The id of the start node.
        :param id_goal: The id of the goal node.
        :return:
        '''
        return self.h_euc(self.pos_from_id(id_start), self.pos_from_id(id_goal))
    
    def id_from_pos(self, pos):
        '''
        Returns a unique id for a given (x,y) position in the map.
        :param pos: The position used to generate the unique id.
        :return: The unique id generated.
        '''
        return (pos[0] - 1) * self.map.shape[0] + pos[1]
        
    def pos_from_id(self, index):
        '''
        Calculates the position of a node within the map using the given index.
        :param index: The index of the node within the map.
        :return: The position (x,y) of the node within the map.
        '''
        index -= 1
        x = floor(index / self.map.shape[0])
        y = index % self.map.shape[1]
        return np.array([x-1, y-1])
        
    def h_euc(self, start, goal):
        '''
        Returns the heuristic value for the given start point to the goal.
        :param start: The start position in the map.
        :param goal: The goal position in the map
        :return: The euclidean distance to the goal.
        '''
        return np.linalg.norm(start-goal)*self.epsilon
    
    def is_valid(self, position):
        '''
        Checks whether the given position is within the bounds and not occupied.
        :param position: The position in the map to check for validity.
        :return: True if the given position is within the bounds and not occupied, False otherwise.
        '''
        # Check for map bounds
        if position[0] < 0 or position[0] >= self.map.shape[0] or \
                position[1] < 0 or position[1] >= self.map.shape[1]:
            return False
        # Check for occupancy of position
        if self.map[position[0], position[1]]:
            return False
        return True
             
    def get_neighbors(self, node):
        '''
        Finds all valid neighbors for a given node.
        :param node: The node for which to find the neighbors.
        :return: A list of nodes that are neighbors of the given node.
        '''
        nbrs = list()
        for i in range(len(self.actions)):
            new_pos = self.actions[i,:]+node.pos
            # Check if new node's position is within the map
            if not self.is_valid(new_pos):
                continue
            # If new position is valid, create a new node using the parent nodes values
            new_id = self.id_from_pos(new_pos)
            nbrs.append(Node(new_id, parent=node.id,
                             g=node.g+self.cost(node.id, new_id), pos=new_pos))
        return nbrs
    
    def get_g(self, index):
        '''
        Returns the g value from a node using open/closed queue.
        :param index: The index of the node the g value is requested from.
        :return: The g value if the specified node.
        '''
        for n in self.open.queue:
            if n.id == index:
                return n.g
        for n in self.closed.queue:
            if n.id == index:
                return n.g
        return None
   
    def print_path(self, path):
        '''
        Inserts the path into the map and prints it.
        :param path: The path to be visualized.
        :return: Nothing.
        '''
        map_tmp = self.map
        for pos in path:
            map_tmp[pos[0], pos[1]] = 2
        print('\nFinal Path:')
        print(map_tmp)
        print('\n(0) identifies a free cell')
        print('(1) identifies an occupied cell')
        print('(2) identifies a cell belonging to the path')

    def calculate_path(self, start, goal, start_orientation=0.0):
        '''
        Calculates the shortest path from start to goal according to ARCThetaA-Star
        :param start_orientation: The initial orientation of the robot.
        :param start: Start position as x/y-coordinates within the map.
        :param goal: Goal position as x/y-coordinates within the map.
        :return: The path as a list of positions within the map from start to goal.
        '''
        
        # Boundary checks
        if not self.is_valid(start):
            print('Start position out of map bounds.')
            return
        if not self.is_valid(goal):
            print('Goal position out of map bounds.')
            return
            
        # Save start/goal pos as node
        n_s = Node(self.id_from_pos(start), pos=start, h=self.h_euc(start, goal),
                   o=start_orientation)
        n_g = Node(self.id_from_pos(goal), pos=goal)
        
        # Set current node equal to start node
        n_cur = n_s
        
        print('Start finding path...')
        
        # Loop until goal reached
        while n_cur.id != n_g.id:
            print(n_cur.pos)
            # Add current node to closed list
            self.closed.put(n_cur)
            # Get a list of all neighboring nodes
            nbrs = self.get_neighbors(n_cur)
            # Loop through neighboring nodes
            for n_nbr in nbrs:
                # Check if open contains neighbor
                if n_nbr in self.open.queue:
                    # Check for line of sight
                    if self.line_of_sight(n_cur.parent, n_nbr.id):
                        # Update parameters to cur parents parameters if line_of_sight
                        n_nbr.parent = n_cur.parent
                        n_nbr.g = self.get_g(n_cur.parent) + \
                                  self.cost(n_cur.parent, n_nbr.id)
                        # Do a normal update with cur values otherwise
                        self.update_node(n_nbr)
                    elif self.line_of_sight(n_cur.id, n_nbr.id):
                        # Do a normal update with cur values otherwise
                        self.update_node(n_nbr)
                # Check if closed does not contain neighbor
                elif n_nbr not in self.closed.queue:
                    if self.line_of_sight(n_cur.parent, n_nbr.id):
                        # Update parameters to cur parents parameters if line_of_sight
                        n_nbr.parent = n_cur.parent
                        n_nbr.g = self.get_g(n_cur.parent) + \
                                  self.cost(n_cur.parent, n_nbr.id)
                    elif not self.line_of_sight(n_cur.id, n_nbr.id):
                        continue
                    # Check angle
                    if not self.arc_check_angle(n_cur, n_nbr):
                        continue
                    # Calculate heuristic value for new node
                    n_nbr.h = self.h_euc(n_nbr.pos, goal)
                    # Add new node to open queue
                    self.open.put(n_nbr)
            
            # Get next
            if self.open.empty():
                print('Error! No Nodes left and goal not reached!')
                return []
            
            n_cur = self.open.get()
        
        print('Finished path search.')
        
        # Follow path back
        path = [n_cur.pos]
        while n_cur.id != n_s.id:
            # Change current node to parent node
            n_cur = self.get_node(n_cur.parent)
            # Append current position to path
            path.append(n_cur.pos)
        
        self.print_path(path)
        
        return path
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
            
            
            
    
        
        