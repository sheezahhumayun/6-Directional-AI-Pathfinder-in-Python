%matplotlib qt 
import matplotlib.pyplot as plt
import numpy as np
import random
from collections import deque
import heapq

class AIPathfinder:
    def __init__(self, size=15, p_dynamic=0.03):
        self.size = size
        self.p_dynamic = p_dynamic
        self.grid = np.zeros((size, size))
        self.start = (size - 2, 1)
        self.target = (1, size - 2)
        self.obstacles = set()
        self._generate_static_walls()
        
    def _generate_static_walls(self):
        mid = self.size // 2
        for r in range(2, self.size - 2):
            self.obstacles.add((r, mid))
            self.grid[r, mid] = -1
