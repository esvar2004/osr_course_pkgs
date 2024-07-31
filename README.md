## Problem 1 - "Solving a 2D motion planning problem by PRM"

### Question: Implement the PRM algorithm described earlier to solve this problem instance. Generate other query instances and environment instances and test your algorithm.

Hint: you may use the function 𝚎𝚗𝚟.𝚌𝚑𝚎𝚌𝚔_𝚌𝚘𝚕𝚕𝚒𝚜𝚒𝚘𝚗(𝚡,𝚢), which returns 𝚃𝚛𝚞𝚎 if the point (𝚡,𝚢) is contained within a triangular obstacles, 𝙵𝚊𝚕𝚜𝚎 otherwise.

### ** Check prm_first_problem.py for solution. environment_2d.py was also altered to include the condition for checking edge collisions.
This is the PRM algorithm that I employed:
1) Generate a random node within the space and check if it lies within one of the triangular objects.
2) If not, append it to the search space.
3) As you append, check if any nodes exist within a certain distance (variable parameter) from that node and try
connecting the two through a graph.
4) If yes, then append that edge to the graph.
5) Apply Dijkstra's algorithm to find the optimal path.

#### Comments:
The initial version of PRM I used was relatively **volatile** and **didn't yield a solution in every case** since the probability of randomly generating nodes around the vertices of the triangles is low. Hence, to make the algorithm more efficient, I **manually generated nodes around the vertices** to ensure that a solution will be found, irrespective of the arrangement of obstacles.

Unlike the previous version, where it was necessary to increase the number of nodes generated to find a possible path, this implementation requires **minimal node creation** and returns a **near-optimal path in every scenario**.

##### Pink -> Vertices / Yellow -> Vertices On Selected Path

#### 100 Nodes Generated w/ Nodes Surrounding Vertices
<img width="900" alt="Screenshot 2024-07-30 at 11 41 15 AM" src="https://github.com/user-attachments/assets/a35d42ba-23bf-40d5-8e89-7ec5a6ac0dff">

#### Only Nodes Surrounding Vertices (Left - Only Optimal Path / Right - Network of Nodes)
<img width="450" alt="Screenshot 2024-07-30 at 11 43 00 AM" src="https://github.com/user-attachments/assets/9b9e650c-7960-43d1-8e97-e4a284df11a6">
<img width="450" alt="Screenshot 2024-07-30 at 11 51 50 AM" src="https://github.com/user-attachments/assets/2797d7c2-80cc-4f00-8f13-08432dd69094">


#### In both scenarios, the optimal path is the same, illustrating that generating nodes around vertices is an intelligent solution to this problem, and appending additional vertices only increases computational costs.
#### Distance of Shortest Path: 14.462547339597087 / Path: [0, 18, 5, 9, 1]


### Performance Under Variable Conditions
#### 20 x 10 - 8 Obstacles (50 Nodes + Triangle Nodes)
<img width="750" alt="Screenshot 2024-07-30 at 11 56 28 AM" src="https://github.com/user-attachments/assets/fc6c6b0f-ada7-4ef1-81ba-696b029d4824">

#### Distance of Shortest Path: 24.395205295102663 / Path: [0, 7, 60, 44, 87, 26, 1]

#### 50 x 25 - 4 Obstacles (50 Nodes + Triangle Nodes)
<img width="750" alt="Screenshot 2024-07-30 at 12 00 42 PM" src="https://github.com/user-attachments/assets/b81a5124-adf7-434b-b4b6-525ad74f27af">

#### Distance of Shortest Path: 50.46195615430966 / Path: [0, 85, 42, 27, 24, 5, 43, 51, 1]

## Problem 2 - "Post-processing a 2D path"

### Question: Implement the above algorithm in Python to post-process the paths found in Section Path planning.

### ** Check prm_second_problem.py for solution.
This is the post-processing path-shortcutting algorithm I employed:
1) Determine the initial path (without collisions) using PRM.
2) For every two consecutive edges, consider intermediary points along them to check if the distance from the initial node to the end node through p1 and p2 (points on the two edges) is lesser than traveling through the middle node.
3) If so, replace the original edge formation with the optimized edge.
4) If not, maintain the original path.
5) Repeat until every pair of consecutive edges is iterated over.

#### Comments:
After analyzing intermediate points along two consecutive edges, we check if the distance from (start -> p1 -> p2 -> end) < (start -> middle -> end). If so, the middle node is removed from the optimal path as the distance can be minimized. If the path can be shortcutted, the new nodes are visualized and the new path is constructed. This particular function is iterative, meaning for each call, it only considers two adjacent edges of the provided path, depending on the value of 'index'.

For this version, I utilized PRM solely, therefore there was no manual generation of nodes near the vertices. Hence, the probability of not finding a solution increases; however, an optimized path will be found in every scenario where a solution is returned.

** For the following images, the green path represents the newly formed path after shortcutting has been applied.

##### Pink -> Vertices / Yellow -> Vertices On Initial Path / Green -> Vertices on Shortcutted Path

#### Path Shortcutting Applied to the Given Environment
<img width="750" alt="Screenshot 2024-07-31 at 9 37 34 AM" src="https://github.com/user-attachments/assets/bb57b5da-181b-45d3-9863-3f557b6cc01d">

#### Distance of Initial Path: 18.441994414350074 / Distance of Path After Shortcutting: 17.68057676911336

#### 20 x 20 - 6 Obstacles (100 Nodes)
<img width="750" alt="Screenshot 2024-07-31 at 9 43 01 AM" src="https://github.com/user-attachments/assets/66176857-067e-428c-914d-45613e432083">

#### Distance of Initial Path: 31.28825663655372 / Distance of Path After Shortcutting: 29.197594718121145

#### 50 x 50 - 10 Obstacles (100 Nodes)
<img width="750" alt="Screenshot 2024-07-31 at 9 45 37 AM" src="https://github.com/user-attachments/assets/eed79068-95c9-4a7f-9eed-1f611eed9cd2">

#### Distance of Initial Path: 69.92832755752275 / Distance of Path After Shortcutting: 67.86777691702287









