# Example of world building, display, and successor computation for the artificial 
# intelligence path-finding lab
#
# Author: Didier Lime
# Date: 2018-10-03

from random import random
from sys import stdout
from collections import deque
from heapq import heappop, heappush

class world:
    # initialise the world
    # L is the number of columns
    # H is the number of lines
    # P is the probability of having a wall in a given tile
    def __init__(self, L, H, P, c=10):
        self.L = L 
        self.H = H
        self.c = c

        # the world is represented by an array with one dimension
        self.w = [0 for i in range(L*H)] # initialise every tile to empty (0)
        self.cost = [1 for i in range(L*H)] # initialise every tile cost to 1

        # add walls in the first and last columns
        for i in range(H):
            self.w[i*L] = 1
            self.w[i*L+L-1] = 1
        
        # add walls in the first and last lines
        for j in range(L):
            self.w[j] = 1
            self.w[(H-1)*L + j] = 1

        for i in range(H):
            for j in range(L):
                # add a wall in this tile with probability P and provided that it is neither
                # the starting tile nor the goal tile 
                if random() < P and not (i == 1 and j == 1) and not (i == H-2 and j == L-2):
                    self.w[i*L+j] = 1
                # set the cost for the tiles in the specified rectangle
                if L//3 <= j < 2*L//3:
                    self.cost[i*L+j] = c

    # display the world
    def display(self):
        for i in range(self.H):
            for j in range(self.L):
                if self.w[i * self.L + j] == 0:
                    if self.cost[i * self.L + j] > 1:
                        stdout.write('\033[92m.\033[0m')  # Green dot for high-cost tiles
                    else:
                        stdout.write('.')
                elif self.w[i * self.L + j] == 1:
                    stdout.write('W')
            print('')

    # compute the successors of tile number i in world w
    def successors(self, i):
        if i < 0 or i >= self.L * self.H or self.w[i] == 1:
            # i is an incorrect tile number (outside the array or on a wall)
            return [] 
        else:
            # look in the four adjacent tiles and keep only those with no wall
            return list(filter(lambda x: self.w[x] != 1, [i - 1, i + 1, i - self.L, i + self.L]))

    # Depth-first search
    # starting from tile number s0, find a path to tile number t
    # return (r, path, total_cost) where r is true if such a path exists, false otherwise
    # and path contains the path if it exists, and total_cost is the total cost of the path
    def dfs(self, s0, t):
        path = []  # Initialize the path as an empty list
        pred = {s0: None}  # Dictionary to store predecessors, starting with s0 having no predecessor
        r = False  # Boolean to indicate if a path is found, initially set to False
        P = set()  # Set of processed vertices
        W = {s0}  # Set of open vertices (waiting to be explored), starting with s0

        while W and not r:  # Continue while there are open vertices and no path is found
            s = W.pop()  # Take an element from the open set
            if s == t:  # If the current vertex is the target
                r = True  # Path is found
            else:
                P.add(s)  # Mark the current vertex as processed
            for s_prime in self.successors(s):  # Iterate over successors of the current vertex
                if s_prime not in P and s_prime not in W:  # If the successor is not processed and not in open set
                    pred[s_prime] = s  # Set the predecessor of the successor
                    W.add(s_prime)  # Add the successor to the open set
        total_cost = 0  # Initialize the total cost
        if r:  # If a path is found
            path = []  # Initialize the path
            step = t  # Start from the target
            while step is not None:  # Trace back from target to start
                path.append(step)  # Add the current step to the path
                step = pred[step]  # Move to the predecessor
            path.reverse()  # Reverse the path to get the correct order from start to target
            # Calculate the total cost
            for node in path:
                total_cost += self.cost[node]
        else:
            total_cost = float('inf')  # No path found, cost is infinite
        return (r, path, total_cost)  # Return the result, the path, and the total cost

    # display the world with the path
    def display_with_path(self, path):
        for i in range(self.H):
            for j in range(self.L):
                if i * self.L + j in path:
                    stdout.write('\033[91m*\033[0m')  # Red star for the path
                elif self.w[i * self.L + j] == 0:
                    if self.cost[i * self.L + j] > 1:
                        stdout.write('\033[92m.\033[0m')  # Green dot for high-cost tiles
                    else:
                        stdout.write('.')
                elif self.w[i * self.L + j] == 1:
                    stdout.write('W')
            print('')

    def bfs(self, s0, t):
        path = []
        pred = {s0: None}
        r = False
        P = set()
        W = deque([s0])

        while W and not r:
            s = W.popleft()
            if s == t:
                r = True
            else:
                P.add(s)
            for s_prime in self.successors(s):
                if s_prime not in P and s_prime not in W:
                    pred[s_prime] = s
                    W.append(s_prime)
        total_cost = 0
        if r:
            path = []
            step = t
            while step is not None:
                path.append(step)
                step = pred[step]
            path.reverse()
            # Calculate the total cost
            for node in path:
                total_cost += self.cost[node]
        else:
            total_cost = float('inf')
        return (r, path, total_cost)
    
    def dijkstra(self, s0, t):
        pred = {s0: None}
        cost = {s: float('inf') for s in range(self.L * self.H)}
        cost[s0] = 0
        r = False
        W = [(0, s0)]  # Priority queue with (cost, node)
        visited = set()

        while W and not r:
            current_cost, s = heappop(W)
            if s == t:
                r = True
            if s not in visited:
                visited.add(s)
                # print(f"Visiting node {s} with cost {current_cost}")
                for s_prime in self.successors(s):
                    new_cost = cost[s] + self.cost[s_prime]
                    if new_cost < cost[s_prime]:
                        cost[s_prime] = new_cost
                        pred[s_prime] = s
                        heappush(W, (new_cost, s_prime))

        path = []
        total_cost = cost[t] if r else float('inf')
        if r:
            step = t
            while step is not None:
                path.append(step)
                step = pred[step]
            path.reverse()
            print(f"Length of the path: {len(path)}")
            print(f"Total cost of the path: {total_cost}")
        return r, path, total_cost
    
    # def heuristic(self, s, t):
    #     x1, y1 = s % self.L, s // self.L
    #     x2, y2 = t % self.L, t // self.L
    #     return abs(x1 - x2) + abs(y1 - y2)

    def heuristic1(self, s, t):
        # s: index of the current tile
        # t: index of the target tile
        # Calculate x and y coordinates of the current tile
        x1, y1 = s % self.L, s // self.L
        # Calculate x and y coordinates of the target tile
        x2, y2 = t % self.L, t // self.L
        # Return the Manhattan distance between the current and target tiles
        return abs(x1 - x2) + abs(y1 - y2)

    def heuristic2(self, s, t):
        # Get x and y coordinates of the current tile s
        x1, y1 = s % self.L, s // self.L
        # Get x and y coordinates of the target tile t
        x2, y2 = t % self.L, t // self.L

        estimated_cost = 0  # Initialize the estimated cost

        # Determine the horizontal step direction (1 for right, -1 for left)
        x_step = 1 if x2 >= x1 else -1
        # Move horizontally from x1 to x2, accumulating the cost
        for x in range(x1, x2, x_step):
            estimated_cost += self.cost[y1 * self.L + x]

        # Determine the vertical step direction (1 for down, -1 for up)
        y_step = 1 if y2 >= y1 else -1
        # Move vertically from y1 to y2 at x2, accumulating the cost
        for y in range(y1, y2, y_step):
            estimated_cost += self.cost[y * self.L + x2]

        return estimated_cost  # Return the total estimated cost

    def a_star1(self, s0, t):
        # Initialize the predecessor dictionary with the start node having no predecessor
        pred = {s0: None}
        # Initialize the cost dictionary with infinity for all nodes
        cost = {s: float('inf') for s in range(self.L * self.H)}
        # The cost to reach the start node is zero
        cost[s0] = 0
        # Initialize the result flag to False
        r = False
        # Initialize the priority queue with the start node and its heuristic
        W = [(self.heuristic1(s0, t), s0)]  # Priority queue with (cost + heuristic, node)
        # Initialize the visited set
        visited = set()

        # Loop while there are nodes to explore and the target hasn't been found
        while W and not r:
            # Pop the node with the lowest priority (cost + heuristic)
            _, s = heappop(W)
            # If the popped node is the target, set the result flag to True
            if s == t:
                r = True
            # If the node hasn't been visited
            if s not in visited:
                # Mark the node as visited
                visited.add(s)
                # Iterate through the successors of the current node
                for s_prime in self.successors(s):
                    # Calculate the new cost to reach the successor
                    new_cost = cost[s] + self.cost[s_prime]
                    # If the new cost is lower than the previously recorded cost
                    if new_cost < cost[s_prime]:
                        # Update the cost for the successor
                        cost[s_prime] = new_cost
                        # Set the predecessor of the successor to the current node
                        pred[s_prime] = s
                        # Calculate the priority using the new cost and heuristic
                        priority = new_cost + self.heuristic1(s_prime, t)
                        # Push the successor onto the priority queue with the updated priority
                        heappush(W, (priority, s_prime))
        # Initialize the path list
        path = []
        # Set the total cost to the cost of the target if reachable, else infinity
        total_cost = cost[t] if r else float('inf')
        if r:
            # Start from the target node
            step = t
            # Trace back the path from target to start using predecessors
            while step is not None:
                path.append(step)
                step = pred[step]
            # Reverse the path to get the correct order from start to target
            path.reverse()
        # Return a tuple indicating if path was found, the path itself, and its total cost
        return r, path, total_cost
    
    def a_star2(self, s0, t):
        # Initialize the predecessor dictionary with the start node having no predecessor
        pred = {s0: None}
        # Initialize the cost dictionary with infinity for all nodes
        cost = {s: float('inf') for s in range(self.L * self.H)}
        # The cost to reach the start node is zero
        cost[s0] = 0
        # Initialize the result flag to False
        r = False
        # Initialize the priority queue with the start node and its heuristic
        W = [(self.heuristic2(s0, t), s0)]  # Priority queue with (cost + heuristic, node)
        # Initialize the visited set
        visited = set()

        # Loop while there are nodes to explore and the target hasn't been found
        while W and not r:
            # Pop the node with the lowest priority (cost + heuristic)
            _, s = heappop(W)
            # If the popped node is the target, set the result flag to True
            if s == t:
                r = True
            # If the node hasn't been visited
            if s not in visited:
                # Mark the node as visited
                visited.add(s)
                # Iterate through the successors of the current node
                for s_prime in self.successors(s):
                    # Calculate the new cost to reach the successor
                    new_cost = cost[s] + self.cost[s_prime]
                    # If the new cost is lower than the previously recorded cost
                    if new_cost < cost[s_prime]:
                        # Update the cost for the successor
                        cost[s_prime] = new_cost
                        # Set the predecessor of the successor to the current node
                        pred[s_prime] = s
                        # Calculate the priority using the new cost and heuristic
                        priority = new_cost + self.heuristic2(s_prime, t)
                        # Push the successor onto the priority queue with the updated priority
                        heappush(W, (priority, s_prime))
        # Initialize the path list
        path = []
        # Set the total cost to the cost of the target if reachable, else infinity
        total_cost = cost[t] if r else float('inf')
        if r:
            # Start from the target node
            step = t
            # Trace back the path from target to start using predecessors
            while step is not None:
                path.append(step)
                step = pred[step]
            # Reverse the path to get the correct order from start to target
            path.reverse()
        # Return a tuple indicating if path was found, the path itself, and its total cost
        return r, path, total_cost

# create a world
w = world(20, 10, 0.2)

print()
print("World:")
print()

# display it 
w.display()

# print the tile numbers of the successors of the starting tile (1, 1)
print(w.successors(w.L + 1))

result1, path1, totalcost1 = w.dfs(21, 178)
result2, path2, totalcost2 = w.bfs(21, 178)
result3, path3, totalcost3 = w.dijkstra(21, 178)
result4, path4, totalcost4 = w.a_star1(21, 178)
result5, path5, totalcost5 = w.a_star2(21, 178)

print()
print("DFS path:")
print()
print("Path found:", result1)
print("Path:", path1)
print("Cost:", totalcost1)
w.display_with_path(path1)
print()
print("BFS path:")
print()
print("Path found:", result2)
print("Path:", path2)
print("Cost:", totalcost2)
w.display_with_path(path2)
print()
print("Dijkstra:")
print()
print("Path found:", result3)
print("Path:", path3)
print("Cost:", totalcost3)
w.display_with_path(path3)
print()
print("A* with heuristic1:")
print()
print("Path found:", result4)
print("Path:", path4)
print("Cost:", totalcost4)
w.display_with_path(path4)
print()
print("A* with heuristic2:")
print()
print("Path found:", result5)
print("Path:", path5)
print("Cost:", totalcost5)
w.display_with_path(path5)