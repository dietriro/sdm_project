#!/usr/bin/env python

import numpy as np

class Node:
    id = -1
    g = 0.0
    h = 0.0
    parent = 0
    x = 0
    y = 0
    o = 0.0
    t = 0.0
    
    def __init__(self, index, pos=np.array([0, 0]), g=0.0, h=0.0, parent=-1, o=0.0, t=0.0):
        self.id = index
        self.g = g
        self.h = h
        self.o = o
        self.t = t
        self.pos = pos
        if parent == -1:
            self.parent = index
        else:
            self.parent = parent
            
    def __cmp__(self, other):
        return cmp(self.g+self.h+self.t*1, other.g+other.h+other.t*1)

    def __eq__(self, other):
        return self.id == other.id
        
    def __ne__(self, other):
        return self.id != other.id
        
    def __hash__(self):
        return self.id
    