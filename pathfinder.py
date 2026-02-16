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
    def bfs(self):
        queue = deque([self.start])
        came_from = {self.start: None}
        explored = set()
        while queue:
            curr = queue.popleft()
            if curr == self.target: return self.reconstruct(came_from), explored
            explored.add(curr)
            for nxt in self.get_neighbors(curr):
                if nxt not in explored and nxt not in queue:
                    came_from[nxt] = curr
                    queue.append(nxt)
            self.visualize(explored, queue, title="BFS - Clockwise Priority")
        return None, explored

    def dfs(self):
        stack = [self.start]
        came_from = {self.start: None}
        explored = set()
        while stack:
            curr = stack.pop()
            if curr == self.target: return self.reconstruct(came_from), explored
            if curr not in explored:
                explored.add(curr)
                for nxt in self.get_neighbors(curr):
                    if nxt not in explored:
                        came_from[nxt] = curr
                        stack.append(nxt)
            self.visualize(explored, stack, title="DFS - Clockwise Priority")
        return None, explored

    def ucs(self):
        pq = [(0, self.start)]
        came_from = {self.start: None}
        cost_so_far = {self.start: 0}
        explored = set()
        while pq:
            curr_cost, curr = heapq.heappop(pq)
            if curr == self.target: return self.reconstruct(came_from), explored
            explored.add(curr)
            for nxt in self.get_neighbors(curr):
                # Diagonal check: cost 1.41 for (1,1) or (-1,-1), else 1.0
                is_diagonal = abs(nxt[0]-curr[0]) + abs(nxt[1]-curr[1]) == 2
                new_cost = cost_so_far[curr] + (1.41 if is_diagonal else 1)
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    came_from[nxt] = curr
                    heapq.heappush(pq, (new_cost, nxt))
            self.visualize(explored, [x[1] for x in pq], title="UCS - Clockwise Priority")
        return None, explored

    def dls(self, limit, visualize=True):
        stack = [(self.start, 0)]
        came_from = {self.start: None}
        explored = set()
        while stack:
            curr, depth = stack.pop()
            if curr == self.target: return self.reconstruct(came_from), explored
            if depth < limit:
                explored.add(curr)
                for nxt in self.get_neighbors(curr):
                    if nxt not in explored:
                        came_from[nxt] = curr
                        stack.append((nxt, depth + 1))
            if visualize: self.visualize(explored, [x[0] for x in stack], title=f"DLS (Limit {limit})")
        return None, explored

    def iddfs(self):
        for depth in range(self.size * self.size):
            path, explored = self.dls(depth)
            if path: return path, explored
        return None, set()

    def bidirectional(self):
        f_frontier, b_frontier = {self.start: None}, {self.target: None}
        f_queue, b_queue = deque([self.start]), deque([self.target])
        f_explored, b_explored = set(), set()
        
        while f_queue and b_queue:
            curr_f = f_queue.popleft()
            f_explored.add(curr_f)
            for nxt in self.get_neighbors(curr_f):
                if nxt in b_explored:
                    return self.reconstruct_bidir(f_frontier, b_frontier, curr_f, nxt), f_explored | b_explored
                if nxt not in f_frontier:
                    f_frontier[nxt] = curr_f
                    f_queue.append(nxt)
            
            curr_b = b_queue.popleft()
            b_explored.add(curr_b)
            for nxt in self.get_neighbors(curr_b):
                if nxt in f_explored:
                    return self.reconstruct_bidir(f_frontier, b_frontier, nxt, curr_b), f_explored | b_explored
                if nxt not in b_frontier:
                    b_frontier[nxt] = curr_b
                    b_queue.append(nxt)
            self.visualize(f_explored | b_explored, list(f_queue) + list(b_queue), title="Bidirectional Search")
        return None, set()
