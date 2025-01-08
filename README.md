![image](https://github.com/user-attachments/assets/89324097-1e06-4ad0-b33b-834a2adf2b11)
![image](https://github.com/user-attachments/assets/e9809433-e218-47bb-a24f-3201c95cd14b)
![image](https://github.com/user-attachments/assets/994342d6-3561-4414-8298-9f20f7bbae6d)

## Research on a new method of pathfinding algorithm for performance improvement
The existing A* algorithm is very well-known and has already been proven to deliver sufficiently good performance.  
However, despite my limited skills and knowledge, I am challenging myself in this project to implement a pathfinding algorithm that performs far better than this.  

## Concept
The basic approach is as follows:  

1. Determine the straight path from the starting point to the destination. (Utilizing Bresenham's Line Algorithm)  
2. While calculating the path, if an obstacle is encountered, immediately stop and recognize the obstacle.  
3. Retrieve the outline information of the obstacle.  
4. Among the outline information, select the optimal point for bypassing the obstacle. (This part is the core)  
5. Combine the starting point and the bypass point to create a kind of waypoint coordinate.  
6. Subsequently, repeat the same process from the bypass point to the destination.  
7. Ultimately, the path is constructed like this: (starting point, waypoint1, waypoint2, ..., waypointN, destination).

*Update* 2025.1.8
1. Detect a collision with an obstacle on the straight path connecting the starting point and the destination.
2. Decide which direction to explore along the obstacle's outline (for now, the side closer to the destination).
3. If the end of the visible outline is reached, search for an appropriate detour point around that outline.
4. Select a detour point where a straight-line movement from the starting point avoids the obstacle, preferably closer to the destination.
5. If the first detour point selection fails, I plan to search in the opposite direction along the outline where the obstacle was first encountered.

This is fundamentally the same concept as the ray concept used in 3D engines.  
However, since I am working in a 2D context and diagonal movement is not allowed, I only made slight adjustments.
