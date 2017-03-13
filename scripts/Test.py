# from ARCThetaStar import ARCThetaStar
from ARCThetaStar_new import ARCThetaStar
from Node import Node
import numpy as np
from math import pi
from Queue import PriorityQueue

map = np.zeros((10,10), int)
map[2:5, 2:6] = 1
map[7, 2:8] = 1
# map[1,1] = 1


map_ = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [0, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                 [0, 0, 0, 0, 1, 1, 1, 0, 0, 0],
                 [1, 1, 1, 0, 1, 1, 1, 1, 0, 0],
                 [0, 0, 0, 0, 1, 1, 1, 1, 0, 0],
                 [0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                 [0, 0, 0, 1, 1, 1, 1, 0, 0, 0],
                 [1, 1, 0, 1, 1, 1, 0, 0, 0, 0],
                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

n = Node(1, pos=np.array([2,2]), o=3.14)
c = Node(2, pos=np.array([2,3]))



a = ARCThetaStar(map_, r_max=2.5, v=1.0)


path = a.calculate_path(np.array([0, 0]), np.array([9, 2]),
                        start_orientation=pi*0.25)