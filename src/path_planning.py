import numpy as np
import heapq

class GridMap:
    def __init__(self, x_min=-5.0, x_max=5.0, y_min=-5.0, y_max=5.0, resolution=0.1, safe_margin=0.25):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.res = resolution
        self.safe_margin = safe_margin
        
        self.width = int(np.ceil((x_max - x_min) / resolution))
        self.height = int(np.ceil((y_max - y_min) / resolution))
        
        # 0 = free, 1 = obstacle
        self.grid = np.zeros((self.width, self.height), dtype=np.int8)

    def to_grid(self, x, y):
        gx = int((x - self.x_min) / self.res)
        gy = int((y - self.y_min) / self.res)
        gx = max(0, min(gx, self.width - 1))
        gy = max(0, min(gy, self.height - 1))
        return gx, gy

    def to_world(self, gx, gy):
        x = self.x_min + gx * self.res + self.res/2
        y = self.y_min + gy * self.res + self.res/2
        return x, y

    def add_box_obstacle(self, cx, cy, w, l):
        """Add a rectangular obstacle with a safety margin dilation"""
        min_x = cx - w/2 - self.safe_margin
        max_x = cx + w/2 + self.safe_margin
        min_y = cy - l/2 - self.safe_margin
        max_y = cy + l/2 + self.safe_margin
        
        g_min_x, g_min_y = self.to_grid(min_x, min_y)
        g_max_x, g_max_y = self.to_grid(max_x, max_y)
        
        g_min_x = max(0, g_min_x)
        g_min_y = max(0, g_min_y)
        g_max_x = min(self.width - 1, g_max_x)
        g_max_y = min(self.height - 1, g_max_y)
        
        for i in range(g_min_x, g_max_x + 1):
            for j in range(g_min_y, g_max_y + 1):
                self.grid[i, j] = 1

    def is_valid(self, gx, gy):
        if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
            return False
        return self.grid[gx, gy] == 0

    def get_neighbors(self, gx, gy):
        neighbors = []
        # 8-way connectivity
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nx, ny = gx + dx, gy + dy
            if self.is_valid(nx, ny):
                # Prevent diagonal cutting through corner obstacles
                if dx != 0 and dy != 0:
                    if not self.is_valid(gx+dx, gy) or not self.is_valid(gx, gy+dy):
                        continue
                cost = 1.414 if dx != 0 and dy != 0 else 1.0
                neighbors.append(((nx, ny), cost))
        return neighbors

def heuristic(a, b):
    # Euclidean distance
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def a_star_search(grid_map, start_w, goal_w):
    start = grid_map.to_grid(*start_w)
    goal = grid_map.to_grid(*goal_w)
    
    if not grid_map.is_valid(*start) or not grid_map.is_valid(*goal):
        print("[Path Planning] Start or Goal is inside an obstacle!")
        return []
        
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}
    
    while frontier:
        _, current = heapq.heappop(frontier)
        
        if current == goal:
            break
            
        for next_node, cost in grid_map.get_neighbors(*current):
            new_cost = cost_so_far[current] + cost
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(next_node, goal)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current
                
    # Reconstruct path
    if goal not in came_from:
        return []
        
    path = []
    current = goal
    while current != start:
        path.append(grid_map.to_world(*current))
        current = came_from[current]
    path.append(grid_map.to_world(*start))
    path.reverse()
    
    # Path smoothing (greedy line of sight) could be added here
    return path

def dijkstra_search(grid_map, start_w, goal_w):
    start = grid_map.to_grid(*start_w)
    goal = grid_map.to_grid(*goal_w)
    
    if not grid_map.is_valid(*start) or not grid_map.is_valid(*goal):
        print("[Path Planning] Start or Goal is inside an obstacle!")
        return []
        
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}
    
    while frontier:
        _, current = heapq.heappop(frontier)
        
        if current == goal:
            break
            
        for next_node, cost in grid_map.get_neighbors(*current):
            new_cost = cost_so_far[current] + cost
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost  # Dijkstra has no heuristic
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current
                
    # Reconstruct path
    if goal not in came_from:
        return []
        
    path = []
    current = goal
    while current != start:
        path.append(grid_map.to_world(*current))
        current = came_from[current]
    path.append(grid_map.to_world(*start))
    path.reverse()
    
    return path

def pure_pursuit(current_pos, current_heading, path, lookahead_dist=0.5):
    """
    Find the lookahead point on the path and calculate steering angle.
    path: List of (x, y) waypoints.
    Returns: (target_pt, heading_error, is_last_point)
    """
    if not path:
        return current_pos, 0.0, True
        
    # Find the furthest point on the path within lookahead_dist
    target_idx = 0
    min_dist = float('inf')
    
    # Find closest point first
    for i, pt in enumerate(path):
        dist = np.linalg.norm(np.array(pt) - np.array(current_pos))
        if dist < min_dist:
            min_dist = dist
            target_idx = i
            
    # Search forward from closest point for the lookahead
    lookahead_idx = target_idx
    for i in range(target_idx, len(path)):
        dist = np.linalg.norm(np.array(path[i]) - np.array(current_pos))
        if dist > lookahead_dist:
            lookahead_idx = i
            break
    else:
        lookahead_idx = len(path) - 1
        
    target_pt = np.array(path[lookahead_idx])
    dx = target_pt[0] - current_pos[0]
    dy = target_pt[1] - current_pos[1]
    
    desired_heading = np.arctan2(dy, dx)
    heading_error = desired_heading - current_heading
    heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
    
    is_last = (lookahead_idx == len(path) - 1)
    
    return target_pt, heading_error, is_last
