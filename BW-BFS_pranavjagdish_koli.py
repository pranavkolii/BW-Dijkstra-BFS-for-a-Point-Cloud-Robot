"""
@file BW-dijkstra_pranavjagdish_koli.py
@author Pranav Jagdish Koli
@brief The Python file containing the Backward BFS algorithm implementation to move a Point robot in a defined 2D grid space with obstacles.
@date 03-14-2026
@copyright Copyright (c) 2026
"""


import numpy as np
import cv2
import time
from collections import deque

# Configuration
WIDTH, HEIGHT = 180, 50
CLEARANCE = 2
SCALE = 5

def check_obstacle(x, y):
    """Checks if node is in 'PK0841' space with 2mm clearance."""
    if x <= CLEARANCE or x >= WIDTH - CLEARANCE or y <= CLEARANCE or y >= HEIGHT - CLEARANCE: 
        return True
    
    # 'P'
    if (15-CLEARANCE <= x <= 20+CLEARANCE) and (15-CLEARANCE <= y <= 35+CLEARANCE): 
        return True
    if ((x-20)**2 + (y-28)**2) <= (7+CLEARANCE)**2 and x >= 20: 
        return True
    
    # 'K'
    if (40-CLEARANCE <= x <= 44+CLEARANCE) and (15-CLEARANCE <= y <= 35+CLEARANCE): 
        return True
    if 44 <= x <= 55 and 25 <= y <= 35 and (y - 0.8*x >= -14-CLEARANCE) and (y - 0.8*x <= -8+CLEARANCE): 
        return True
    if 44 <= x <= 55 and 15 <= y <= 25 and (y + 0.8*x >= 55-CLEARANCE) and (y + 0.8*x <= 61+CLEARANCE): 
        return True
    
    # '0'
    if (6-CLEARANCE)**2 <= (x-75)**2 + (y-25)**2 <= (10+CLEARANCE)**2: 
        return True
    
    # '8'
    if (5-CLEARANCE)**2 <= (x-100)**2 + (y-31)**2 <= (8+CLEARANCE)**2: 
        return True
    if (5-CLEARANCE)**2 <= (x-100)**2 + (y-19)**2 <= (8+CLEARANCE)**2: 
        return True
    
    # '4'
    if (135-CLEARANCE <= x <= 139+CLEARANCE) and (15-CLEARANCE <= y <= 35+CLEARANCE): 
        return True
    if (125-CLEARANCE <= x <= 139+CLEARANCE) and (21-CLEARANCE <= y <= 25+CLEARANCE): 
        return True
    if 125 <= x <= 135 and 25 <= y <= 35 and (y + x >= 157-CLEARANCE) and (y + x <= 163+CLEARANCE): 
        return True
    
    # '1'
    if (160-CLEARANCE <= x <= 164+CLEARANCE) and (15-CLEARANCE <= y <= 35+CLEARANCE): 
        return True
    
    return False

def backward_bfs(start, goal):
    """Standard FIFO queue search from Goal to Start."""
    open_list = deque([goal])
    parent_map = {goal: None}
    explored = []
    start_time = time.time()
    
    # BFS Loop
    while open_list:
        curr = open_list.popleft()
        explored.append(curr)
        
        if curr == start:
            print(f"Goal Found! Runtime: {time.time()-start_time:.2f}s")
            return backtracking(parent_map, start), explored
            
        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1), (1,-1), (-1,-1)]:
            neighbor = (curr[0]+dx, curr[1]+dy)
            if 0 <= neighbor[0] < WIDTH and 0 <= neighbor[1] < HEIGHT:
                if neighbor not in parent_map and not check_obstacle(*neighbor):
                    parent_map[neighbor] = curr
                    open_list.append(neighbor)
    return None, explored

def backtracking(parents, start):
    """ Backtrack from Start to Goal using parent map."""
    path, curr = [], start
    while curr is not None:
        path.append(curr)
        curr = parents[curr]
    return path

def visualization(path, explored, title):
    """ Visualize the explored nodes and final path using OpenCV."""
    canvas = np.zeros((HEIGHT*SCALE, WIDTH*SCALE, 3), dtype=np.uint8)
    for i in range(WIDTH):
        for j in range(HEIGHT):
            if check_obstacle(i, j): 
                cv2.circle(canvas, (i*SCALE, (HEIGHT-j)*SCALE), 1, (0,0,255), -1)
    
    for i, node in enumerate(explored):
        cv2.circle(canvas, (node[0]*SCALE, (HEIGHT-node[1])*SCALE), 1, (255,0,0), -1)
        if i % 200 == 0: 
            cv2.imshow(title, canvas)
            cv2.waitKey(1)
            
    if path:
        for node in path:
            cv2.circle(canvas, (node[0]*SCALE, (HEIGHT-node[1])*SCALE), 2, (0,255,0), -1)
            cv2.imshow(title, canvas)
            cv2.waitKey(20)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Main
if __name__ == "__main__":
    sx, sy = map(int, input("Start x y: ").split())
    gx, gy = map(int, input("Goal x y: ").split())
    if check_obstacle(sx, sy) or check_obstacle(gx, gy):
        print("Invalid Coordinates")
    else:
        path, explored = backward_bfs((sx, sy), (gx, gy))
        visualization(path, explored, "Backward BFS")
