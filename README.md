## Problem 1 - "Solving a 2D motion planning problem by PRM"

### Question: Implement the PRM algorithm described earlier to solve this problem instance. Generate other query instances and environment instances and test your algorithm.

Hint: you may use the function 𝚎𝚗𝚟.𝚌𝚑𝚎𝚌𝚔_𝚌𝚘𝚕𝚕𝚒𝚜𝚒𝚘𝚗(𝚡,𝚢), which returns 𝚃𝚛𝚞𝚎 if the point (𝚡,𝚢) is contained within a triangular obstacles, 𝙵𝚊𝚕𝚜𝚎 otherwise.

This is the PRM algorithm that I employed:
1) Generate a random node and check if it lies within the triangular objects.
2) If not, append it to the search space.
3) As you append, check if any nodes exist within a certain distance (variable parameter) from that node and try
connecting the two through a graph.
4) If yes, then append that edge to the graph.
5) Apply Dijkstra's algorithm to find the optimal path.

The initial version of PRM I used was relatively **volatile** and **didn't yield a solution in every case** since the probability of randomly generating nodes around the vertices of the triangles is low. Hence, to make the algorithm more efficient, I **manually generated nodes around the vertices** to ensure that a solution will be found, irrespective of the arrangement of obstacles.

Unlike the previous version, where it was a necessity to increase the number of nodes generated to find a possible path, this implementation requires **minimal node creation** and returns a **near-optimal path in every scenario**.

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




