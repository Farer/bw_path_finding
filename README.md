![ezgif-7-1943443c6d](https://github.com/user-attachments/assets/ad4c7369-b042-41b7-943f-40f74c1dfcb8)

> [!NOTE]
> This source code has not been refactored.
>
> Pull requests are more than welcome.

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

This is fundamentally the same concept as the ray concept used in 3D engines.  
However, since I am working in a 2D context and diagonal movement is not allowed, I only made slight adjustments.

## Update 2025.1.8
1. Detect a collision with an obstacle on the straight path connecting the starting point and the destination.
2. Decide which direction to explore along the obstacle's outline (for now, the side closer to the destination).
3. If the end of the visible outline is reached, search for an appropriate detour point around that outline.
4. Select a detour point where a straight-line movement from the starting point avoids the obstacle, preferably closer to the destination.
5. If the first detour point selection fails, I plan to search in the opposite direction along the outline where the obstacle was first encountered.

## Update 2025.1.23
In general, for distances beyond a reasonable range, the following algorithm is applied:
- Calculate a straight path between the starting point and the destination using **Bresenham's line algorithm**.
- If an obstacle is encountered during the calculation, immediately stop the computation.
- Check the obstacles connected to the left and right of the detected obstacle based on its position.
- If the obstacles in a specific direction touch the map's boundary, exclude that direction.
- If both sides have open paths, calculate the positions of the obstacles visible at both ends in the field of view.
- Identify areas of available space near the ends and choose the corner with the largest available space as the next move target.
- If the available spaces are equal, choose the location closest to the starting point as the next move target.
- In certain cases, the path may get stuck at a sharp edge of an obstacle. In this situation, select the location near the obstacle where both the starting point and destination are close as the next move target.

If there are no obstacles from the beginning, the algorithm only needs to fire a single ray using **Bresenham's line algorithm**, ensuring the fastest performance.
As the distance increases, the algorithm exhibits significantly higher performance compared to other algorithms (which, theoretically, is quite natural).
By repeating the above process, the algorithm gradually finds its way to the destination.

For distances within a 10 x 10 range, a more simplified algorithm is applied:
- This algorithm accounts for cases where the path is in close contact with various obstacles in a narrow area.
- Starting from a distance of N, the algorithm decreases the distance by -1 while applying a simple method to navigate around obstacles.

In this case, the algorithm is simpler but may sometimes perform slower than A*.

## Branches
- main : BW algorithm
- Anya
- Tangent Bug

## Projects that use this algorithm
[Breathing World](https://breathingworld.com)
