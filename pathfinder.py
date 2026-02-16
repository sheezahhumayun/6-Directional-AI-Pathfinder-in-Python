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
    def get_neighbors(self, node):
        r, c = node
        directions = [
            (-1, 0),  # 1. Up
            (0, 1),   # 2. Right
            (1, 0),   # 3. Bottom
            (1, 1),   # 4. Bottom-Right (Diagonal)
            (0, -1),  # 5. Left
            (-1, -1)  # 6. Top-Left (Diagonal)
        ]
        
        neighbors = []
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.size and 0 <= nc < self.size and (nr, nc) not in self.obstacles:
                neighbors.append((nr, nc))
        return neighbors

    def spawn_dynamic_obstacle(self, current_path):
        if random.random() < self.p_dynamic:
            r, c = random.randint(0, self.size-1), random.randint(0, self.size-1)
            if (r, c) not in [self.start, self.target] and (r, c) not in self.obstacles:
                self.obstacles.add((r, c))
                self.grid[r, c] = -1
                print(f"!!! Dynamic Obstacle spawned at {r, c} !!!")
                
                if current_path and (r, c) in current_path:
                    print("Path Blocked! Re-planning...")
                    return True
        return False

    def visualize(self, explored, frontier, path=None, title="AI Pathfinding"):
        plt.clf()
        display_grid = np.copy(self.grid)
        for node in explored: display_grid[node] = 2  
        for node in frontier: 
            if isinstance(node, tuple): display_grid[node] = 1 
        if path:
            for node in path: display_grid[node] = 3 
        
        display_grid[self.start] = 4 
        display_grid[self.target] = 5 
        
        plt.imshow(display_grid, cmap='tab20c')
        plt.title(title)
        plt.pause(0.05)

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

    def reconstruct(self, came_from):
        path, curr = [], self.target
        while curr:
            path.append(curr); curr = came_from.get(curr)
        return path[::-1]

    def reconstruct_bidir(self, f_frontier, b_frontier, f_meet, b_meet):
        p1, curr = [], f_meet
        while curr: p1.append(curr); curr = f_frontier.get(curr)
        p2, curr = [], b_meet
        while curr: p2.append(curr); curr = b_frontier.get(curr)
        return p1[::-1] + p2

if __name__ == "__main__":
    print("Select Algorithm:\n1. BFS\n2. DFS\n3. UCS\n4. DLS\n5. IDDFS\n6. Bidirectional")
    choice = input("Enter number: ")
    
    app = AIPathfinder(size=12)
    algos = {"1": app.bfs, "2": app.dfs, "3": app.ucs, "4": lambda: app.dls(10), "5": app.iddfs, "6": app.bidirectional}
    
    if choice in algos:
        path, explored = algos[choice]()
        if path:
            for i in range(len(path)):
                if app.spawn_dynamic_obstacle(path[i:]):
                    print("Re-calculating due to dynamic hurdle...")
                    break
            app.visualize(explored, [], path=path)
            print("Path Found!")
            plt.show()
        else:
            print("No path possible.")
