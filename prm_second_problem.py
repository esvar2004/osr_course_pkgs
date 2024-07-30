import numpy as np
import pylab as pl
import sys
sys.path.append('osr_examples/scripts/')
import environment_2d
import random
import math
pl.ion()
np.random.seed(4)

"""
PRM Algorithm:
1) Generate a random node and check if it lies within the triangular objects.
2) If not, append it to the search space.
3) As you append, check if any nodes exist within a certain distance from that node and try
connecting the two through a graph.
4) If yes, then append that edge to the graph.
5) Apply Dijkstra's algorithm or A* search to find the optimal path.
"""

"""
This function initializes coordinates around the vertices of the obstacles,
so that we're able to navigate perfectly around them whilst minimizing straight-line distance.
This also ensures that a solution will be found in all cases, increasing the performance of a
probabilistic representation.
"""
def triangle_coord(x1, y1, dist_var):
  num = 0
  if(not env.check_collision(x1 + dist_var * x_len , y1 + dist_var * y_len)):
    graph.append([x1 + dist_var * x_len , y1 + dist_var * y_len])
    env.plot_coordinate(x1 + dist_var * x_len, y1 + dist_var * y_len, "om")
    num += 1

  if(not env.check_collision(x1 + dist_var * x_len , y1 - dist_var * y_len)):
    graph.append([x1 + dist_var * x_len , y1 - dist_var * y_len])
    env.plot_coordinate(x1 + dist_var * x_len, y1 - dist_var * y_len, "om")
    num += 1

  if(not env.check_collision(x1 - dist_var * x_len , y1 + dist_var * y_len)):
    graph.append([x1 - dist_var * x_len , y1 + dist_var * y_len])
    env.plot_coordinate(x1 - dist_var * x_len, y1 + dist_var * y_len, "om")
    num += 1

  if(not env.check_collision(x1 - dist_var * x_len , y1 - dist_var * y_len)):
    graph.append([x1 - dist_var * x_len , y1 - dist_var * y_len])
    env.plot_coordinate(x1 - dist_var * x_len, y1 - dist_var * y_len, "om")
    num += 1
  
  return num

# Distance between two nodes.
def distance(x1, y1, x2, y2):
  return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

# Returns the shortest path.
def dijkstra(adj_matrix, start_node, target_node, env):
    num_nodes = len(adj_matrix)
    
    # Initialize distances with infinity and set the distance to the start node as 0
    distances = [float('inf')] * num_nodes
    distances[start_node] = 0
    
    # Predecessor Array
    predecessors = [-1] * num_nodes
    
    # Visited Array
    visited = [False] * num_nodes
    
    for _ in range(num_nodes):
        # Select the unvisited node with the smallest distance
        min_distance = float('inf')
        min_index = -1
        for i in range(num_nodes):
            if not visited[i] and distances[i] < min_distance:
                min_distance = distances[i]
                min_index = i
        
        if min_index == -1:
            break
        
        # Mark the selected node as visited
        visited[min_index] = True
        
        # Update the distances of the neighboring nodes of the selected node
        for i in range(num_nodes):
            if adj_matrix[min_index][i] > 0 and not visited[i]:
                new_distance = distances[min_index] + adj_matrix[min_index][i]
                if new_distance < distances[i]:
                    distances[i] = new_distance
                    predecessors[i] = min_index
    
    # Reconstruct the path from start_node to target_node
    path = []
    current_node = target_node
    # prev_node = current_node
    while current_node != -1:
        env.plot_coordinate(graph[current_node][0], graph[current_node][1], "oy")
        path.insert(0, current_node)
        prev_node = current_node
        current_node = predecessors[current_node]
        if(current_node == -1):
          break
        pl.plot([graph[current_node][0], graph[prev_node][0]], [graph[current_node][1], graph[prev_node][1]], "b" , linewidth = 1)

    
    if distances[target_node] == float('inf'):
        return float('inf'), []  # No path found
    
    print('Distance of Shortest Path:', distances[target_node])
    print('Path:', path)
    
    return distances[target_node], path

def interpolate_points(p1, p2, num_points=10):
    """Interpolates points between p1 and p2."""
    return [(p1[0] + i * (p2[0] - p1[0]) / num_points, p1[1] + i * (p2[1] - p1[1]) / num_points) for i in range(1, num_points + 1)]

def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def path_shortcutting(path, graph, env):
    i = 0
    while i < len(path) - 2:
        start = graph[path[i]]
        middle = graph[path[i + 1]]
        end = graph[path[i + 2]]

        # Interpolate points on the first and second edges
        points_on_first_edge = interpolate_points(start, middle)
        points_on_second_edge = interpolate_points(middle, end)

        for p1 in points_on_first_edge:
            for p2 in points_on_second_edge:
                if (not env.check_intersection(p1[0], p1[1], p2[0], p2[1])):
                    #print(p1[0], p1[1], p2[0], p2[1])
                    direct_dist = euclidean_distance(start[0], start[1], p1[0], p1[1]) + euclidean_distance(p1[0], p1[1], p2[0], p2[1]) + euclidean_distance(p2[0], p2[1], end[0], end[1])
                    current_dist = euclidean_distance(start[0], start[1], middle[0], middle[1]) + euclidean_distance(middle[0], middle[1], end[0], end[1])
                    # print(direct_dist, current_dist)
                    if direct_dist < current_dist:
                        # Replace the two edges with the direct edge
                        #print("Yes")
                        # env.plot_coordinate(p1[0], p1[1], "og")
                        # env.plot_coordinate(p2[0], p2[1], "og")
                        current_dist = direct_dist
                        # new_path = path[:i + 1] + [path[i + 2]]
                        # path = new_path
                        # print(path)
                        # shortcut_found = True
        
        i += 1
    return path

#Characteristics of the Environment
x_len = 10
y_len = 6
num_obstacles = 5
dist_var = 0.01 #Degree of Altering Distance of Coordinate from Triangular Vertex
num = 2

graph = [] # Two-Dimensional Graph of Vertices

env = environment_2d.Environment(x_len, y_len, num_obstacles)
# pl.clf()
env.plot()
q = env.random_query()
if q is not None:
  x_start, y_start, x_goal, y_goal = q
  graph.append([x_start, y_start])
  graph.append([x_goal, y_goal])
  env.plot_query(x_start, y_start, x_goal, y_goal)

#Populating Coordinates Around Triangular Vertices
"""
for ob in env.obs:
  num += triangle_coord(ob.x0, ob.y0, dist_var)
  num += triangle_coord(ob.x1, ob.y1, dist_var)
  num += triangle_coord(ob.x2, ob.y2, dist_var)
"""

size = 100 + num # Size of Adjacency Matrix (including Start and End Nodes)
adj_mat = [[0] * size for _ in range(size)]

#Populating Remaining Coordinates
while num < size:
  x_crd = random.random() * x_len
  y_crd = random.random() * y_len

  #Checks for valid coordinate
  if(not env.check_collision(x_crd, y_crd)):
    graph.append([x_crd, y_crd]) # X and Y coordinate, followed by indices of attainable coordinates
    env.plot_coordinate(x_crd, y_crd, "om")
    num += 1
  else:
    continue

#Populating Adjacency Matrix with Distance Values
for i in range(0, size):
   coord = graph[i]

   for j in range(i + 1, size):
    temp = graph[j]
    dist = distance(temp[0], temp[1], coord[0], coord[1])

    if(not env.check_intersection(temp[0], temp[1], coord[0], coord[1]) and dist < 10):
        # pl.plot([temp[0], coord[0]], [temp[1], coord[1]], "g" , linewidth = 1)
        adj_mat[i][j] = dist #Populating Adjacency Matrix
        adj_mat[j][i] = dist
      
distance, initial_path = dijkstra(adj_mat, 0, 1, env)
optimized_path = path_shortcutting(initial_path, graph, env)

print(optimized_path)

pl.show(block = True)