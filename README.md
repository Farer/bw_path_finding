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
7. Ultimately, the path is constructed like this: (starting point, waypoint1, waypoint2, ..., waypoint3, destination).  

This is fundamentally the same concept as the ray concept used in 3D engines.  
However, since I am working in a 2D context and diagonal movement is not allowed, I only made slight adjustments.  
