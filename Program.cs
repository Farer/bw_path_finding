namespace bw_path_finding;

public class PathFinder
{
    /// <summary>
    /// Finds a path from the start point to the end point, stopping immediately upon encountering an obstacle and
    /// efficiently identifying valid detour points by exploring the obstacle's edge.
    /// </summary>
    /// <param name="start">The starting point.</param>
    /// <param name="end">The destination point.</param>
    /// <param name="obstacles">A set of obstacle coordinates.</param>
    /// <returns>A tuple containing the path taken before interruption (if any),
    /// and a list of valid detour points around the encountered obstacle.</returns>
    public (List<(int X, int Y)> Path, List<(int X, int Y)> ValidDetourPoints) FindPathUntilObstacle(
            (int X, int Y) start,
            (int X, int Y) end,
            HashSet<(int X, int Y)> obstacles,
            bool isReverse = false,
            int mapSize = 15
        )
    {
        var path = new List<(int X, int Y)>();
        int currentX = start.X;
        int currentY = start.Y;

        int deltaX = Math.Abs(end.X - start.X);
        int deltaY = Math.Abs(end.Y - start.Y);

        int stepX = start.X < end.X ? 1 : -1;
        int stepY = start.Y < end.Y ? 1 : -1;

        int error = deltaX - deltaY;

        while (true)
        {
            path.Add((currentX, currentY));

            if (obstacles.Contains((currentX, currentY)) && (currentX, currentY) != start) // Obstacle encountered
            {
                // Explore the edge of the encountered obstacle to find detour points.
                var validDetourPoints = FindDetourPointsAlongEdge(start, end, (currentX, currentY), obstacles, isReverse, mapSize);
                return (path, validDetourPoints);
            }

            if (currentX == end.X && currentY == end.Y)
            {
                break;
            }

            int doubleError = 2 * error;
            if (doubleError > -deltaY)
            {
                error -= deltaY;
                currentX += stepX;
            }
            else if (doubleError < deltaX)
            {
                error += deltaX;
                currentY += stepY;
            }
        }

        return (path, []); // Reached the destination
    }

    /// <summary>
    /// Explores the edge of the obstacle encountered from the starting point to find possible detour points.
    /// </summary>
    /// <param name="start">The starting point.</param>
    /// <param name="end">The destination point.</param>
    /// <param name="obstacleHit">The coordinates of the first encountered obstacle.</param>
    /// <param name="obstacles">A set of all obstacle coordinates.</param>
    /// <returns>A list of valid detour points.</returns>
    private List<(int X, int Y)> FindDetourPointsAlongEdge(
            (int X, int Y) start,
            (int X, int Y) end,
            (int X, int Y) obstacleHit,
            HashSet<(int X, int Y)> obstacles,
            bool isReverse = false,
            int mapSize = 15
        )
    {
        var validDetourPoints = new List<(int X, int Y)>();
        var visitedEdges = new HashSet<(int X, int Y)>();
        var queue = new Queue<(int X, int Y)>();

        // Add the exterior tiles of the first encountered obstacle to the queue.
        foreach (var neighbor in GetAdjacentTiles(obstacleHit))
        {
            if (!obstacles.Contains(neighbor))
            {
                queue.Enqueue(obstacleHit); // Start exploration from the obstacle hit
                break;
            }
        }

        if (queue.Count == 0) return []; // No way to detour if completely surrounded

        var currentEdge = queue.Dequeue();

        var directionX = 0;
        var directionY = 0;
        var possibleDirections = new List<(int, int)>();

        if (!isReverse)
        {
            // Normal exploration: prioritize direction towards the destination
            directionX = Math.Sign(end.X - currentEdge.X);
            directionY = Math.Sign(end.Y - currentEdge.Y);
        }
        else
        {
            // Reverse exploration: prioritize direction towards the starting point
            directionX = Math.Sign(start.X - currentEdge.X);
            directionY = Math.Sign(start.Y - currentEdge.Y);
        }

        // Configure possibleDirections dynamically based on current movement
        if (directionX != 0 && directionY != 0)
        {
            // Diagonal movement: prioritize diagonal, then X and Y
            possibleDirections = new List<(int, int)>
            {
                (directionX, directionY),    // Diagonal towards target
                (directionX, 0),            // Horizontal
                (0, directionY),            // Vertical
                (-directionX, directionY),  // Opposite diagonal
                (directionX, -directionY),  // Opposite diagonal
                (-directionX, 0),           // Opposite horizontal
                (0, -directionY)            // Opposite vertical
            };
        }
        else if (directionX != 0)
        {
            // Horizontal movement: prioritize X-axis, then diagonal
            possibleDirections = new List<(int, int)>
            {
                (directionX, 0),            // Horizontal
                (directionX, 1),            // Diagonal down
                (directionX, -1),           // Diagonal up
                (0, 1),                     // Vertical down
                (0, -1),                    // Vertical up
                (-directionX, 1),           // Opposite diagonal down
                (-directionX, -1),          // Opposite diagonal up
                (-directionX, 0)            // Opposite horizontal
            };
        }
        else if (directionY != 0)
        {
            // Vertical movement: prioritize Y-axis, then diagonal
            possibleDirections = new List<(int, int)>
            {
                (0, directionY),            // Vertical
                (1, directionY),            // Diagonal right
                (-1, directionY),           // Diagonal left
                (1, 0),                     // Horizontal right
                (-1, 0),                    // Horizontal left
                (1, -directionY),           // Opposite diagonal right
                (-1, -directionY),          // Opposite diagonal left
                (0, -directionY)            // Opposite vertical
            };
        }
        else
        {
            // Default fallback (stationary case)
            possibleDirections = new List<(int, int)>
            {
                (1, 0), (0, 1), (-1, 0), (0, -1),
                (1, 1), (-1, 1), (1, -1), (-1, -1)
            };
        }

        // Filter directions to ensure they are within map bounds
        possibleDirections = possibleDirections.Where(dir =>
            IsWithinMapBounds(currentEdge.X + dir.Item1, currentEdge.Y + dir.Item2, mapSize, mapSize)
        ).ToList();


        var startEdge = currentEdge;
        var currentPoint = startEdge;

        while (true)
        {
            var isReachable = IsReachable(start, currentPoint, obstacles);
            if (isReachable && !validDetourPoints.Contains(currentPoint))
            {
                validDetourPoints.Add(currentPoint);
            }

            bool foundNextEdge = false;
            foreach (var (dx, dy) in possibleDirections)
            {
                var nextEdgeCandidate = (currentPoint.X + dx, currentPoint.Y + dy);
                if (obstacles.Contains(nextEdgeCandidate) && visitedEdges.Add(nextEdgeCandidate))
                {
                    currentPoint = nextEdgeCandidate;
                    foundNextEdge = true;
                    break;
                }
            }

            if (!foundNextEdge) break; // No more edges to explore

            // If the current edge is not reachable from the start, the previous one might be a valid detour point
            if (!IsReachable(start, currentPoint, obstacles))
            {
                break;
            }

            if (currentPoint == startEdge && validDetourPoints.Count > 0) break; // Back to the starting edge
        }

        return validDetourPoints;
    }

    private bool IsWithinMapBounds(int x, int y, int mapWidth, int mapHeight)
    {
        return x >= 0 && x < mapWidth && y >= 0 && y < mapHeight;
    }


    /// <summary>
    /// Returns the adjacent tiles of a given tile.
    /// </summary>
    /// <param name="tile">The reference tile coordinates.</param>
    /// <returns>An enumerable of adjacent tile coordinates.</returns>
    private static IEnumerable<(int X, int Y)> GetAdjacentTiles((int X, int Y) tile)
    {
        yield return (tile.X - 1, tile.Y);
        yield return (tile.X + 1, tile.Y);
        yield return (tile.X, tile.Y - 1);
        yield return (tile.X, tile.Y + 1);
        yield return (tile.X - 1, tile.Y - 1);
        yield return (tile.X - 1, tile.Y + 1);
        yield return (tile.X + 1, tile.Y - 1);
        yield return (tile.X + 1, tile.Y + 1);
    }

    /// <summary>
    /// Finds the optimal detour point near the selected edge that is reachable from the start and closer to the goal.
    /// </summary>
    /// <param name="start">The starting point coordinates.</param>
    /// <param name="goal">The goal point coordinates.</param>
    /// <param name="selectedEdge">The selected edge point coordinates.</param>
    /// <param name="currentPath">The current path taken.</param>
    /// <param name="obstacles">The set of obstacles.</param>
    /// <returns>The optimal detour point, or null if none is found.</returns>
    public (int X, int Y)? FindOptimalDetourPoint(
            (int X, int Y) start,
            (int X, int Y) goal,
            (int X, int Y) selectedEdge,
            List<(int X, int Y)> currentPath,
            HashSet<(int X, int Y)> obstacles,
            bool isReverse = false
        )
    {
        var potentialDetours = new HashSet<(int X, int Y)>(
            GetAdjacentTiles(selectedEdge).Where(tile => !obstacles.Contains(tile))
        );
        // foreach (var point in currentPath.Where(p => !obstacles.Contains(p)))
        // {
        //     potentialDetours.Add(point);
        // }

        (int X, int Y)? bestDetour = null;
        if(!isReverse) {
            double minDistanceToGoal = double.MaxValue;

            foreach (var detour in potentialDetours)
            {
                var isReachable = IsReachable(start, detour, obstacles);
                if (isReachable)
                {
                    double distanceToGoal = CalculateDistance(detour, goal);
                    if (distanceToGoal < minDistanceToGoal)
                    {
                        minDistanceToGoal = distanceToGoal;
                        bestDetour = detour;
                    }
                }
            }
        }
        else {
            double maxDistanceFromStart = double.MinValue;

            foreach (var detour in potentialDetours)
            {
                var isReachable = IsReachable(start, detour, obstacles);
                if (isReachable)
                {
                    double distanceFromStart = CalculateDistance(detour, start);
                    if (distanceFromStart > maxDistanceFromStart)
                    {
                        maxDistanceFromStart = distanceFromStart;
                        bestDetour = detour;
                    }
                }
            }
        }

        return bestDetour;
    }

    // Helper function to calculate the Euclidean distance between two points.
    private static double CalculateDistance((int X, int Y) p1, (int X, int Y) p2)
    {
        int dx = p1.X - p2.X;
        int dy = p1.Y - p2.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    /// <summary>
    /// Checks if the end point is reachable from the start point without crossing any obstacles using a straight-line path (Bresenham's line algorithm).
    /// </summary>
    /// <param name="start">The starting point coordinates.</param>
    /// <param name="end">The destination point coordinates.</param>
    /// <param name="obstacles">A hash set of obstacle coordinates for quick lookup.</param>
    /// <returns>True if the end point is reachable, false otherwise.</returns>
    private bool IsReachable((int X, int Y) start, (int X, int Y) end, HashSet<(int X, int Y)> obstacles)
    {
        int x = start.X;
        int y = start.Y;
        int dx = Math.Abs(end.X - start.X);
        int dy = Math.Abs(end.Y - start.Y);
        int sx = start.X < end.X ? 1 : -1;
        int sy = start.Y < end.Y ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            if (obstacles.Contains((x, y)) && (x, y) != end)
            {
                return false;
            }

            if (x == end.X && y == end.Y)
            {
                return true;
            }

            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x += sx;
            }
            else if (e2 < dx)
            {
                err += dx;
                y += sy;
            }

            // Safety break
            if (Math.Abs(x - start.X) > Math.Abs(end.X - start.X) + 1 || Math.Abs(y - start.Y) > Math.Abs(end.Y - start.Y) + 1)
            {
                return false;
            }
        }
    }
}

class Program
{
    static void Main()
    {
        // var start = (X: 0, Y: 0); var goal = (X: 0, Y: 14);
        // var start = (X: 0, Y: 0); var goal = (X: 5, Y: 7);
        // var start = (X: 5, Y: 7); var goal = (X: 0, Y: 14);
        // var start = (X: 5, Y: 8); var goal = (X: 0, Y: 14);
        var start = (X: 5, Y: 9); var goal = (X: 0, Y: 14);
        // var start = (X: 4, Y: 10); var goal = (X: 0, Y: 14);
        // var start = (X: 4, Y: 11); var goal = (X: 0, Y: 14);

        // var start = (X: 0, Y: 0); var goal = (X: 8, Y: 6);
        // var start = (X: 8, Y: 6); var goal = (X: 13, Y: 14);
        // var start = (X: 8, Y: 6); var goal = (X: 10, Y: 7);
        // var start = (X: 10, Y: 7); var goal = (X: 13, Y: 14);

        var obstacles = new HashSet<(int X, int Y)>
        {
                (1, 6),     (2, 6),
        (0, 7), (1, 7),     (2, 7),     (3, 7),     (4, 7),
                (1, 8),     (2, 8),     (3, 8),     (4, 8),
                (1, 9),     (2, 9),     (3, 9),
                                        (3, 10)
        };

        var pathFinder = new PathFinder();
        var mapSize = 15;

        var (path, validSearchEdges) = pathFinder.FindPathUntilObstacle(start, goal, obstacles, false, mapSize);

        if (validSearchEdges.Count == 0)
        {
            DisplayMap(mapSize, start, goal, obstacles, path, null, null);
        }
        else if (validSearchEdges.Count > 0)
        {
            bool isReverse = false;
            var (X, Y) = validSearchEdges.Last();
            if (X <= 0 || X >= mapSize - 1)
            {
                isReverse = true;
                (path, validSearchEdges) = pathFinder.FindPathUntilObstacle(start, goal, obstacles, true, mapSize);
                if (validSearchEdges.Count == 0)
                {
                    DisplayMap(mapSize, start, goal, obstacles, path, null, null);
                }
            }
            var bestEdgePoint = validSearchEdges.Last();
            (int X, int Y)? optimalDetourPoint = null;
            if (bestEdgePoint != (-1, -1))
            {
                optimalDetourPoint = pathFinder.FindOptimalDetourPoint(start, goal, bestEdgePoint, path, obstacles, isReverse);
                Console.WriteLine($"Optimal detour point: {optimalDetourPoint}");
            }

            List<(int X, int Y)>? finalEdges = bestEdgePoint != (-1, -1) ? [bestEdgePoint] : null;

            DisplayMap(mapSize, start, goal, obstacles, path, finalEdges, optimalDetourPoint);
        }
    }

    static void DisplayMap(int size, (int X, int Y) start, (int X, int Y) goal, HashSet<(int X, int Y)> obstacles, List<(int X, int Y)> path, List<(int X, int Y)>? edges, (int X, int Y)? detourPoint)
    {
        char[,] map = new char[size, size];

        // Initialize map
        for (int y = 0; y < size; y++)
        {
            for (int x = 0; x < size; x++)
            {
                map[y, x] = '.';
            }
        }

        map[start.Y, start.X] = 'S';
        map[goal.Y, goal.X] = 'E';

        foreach (var (X, Y) in obstacles)
        {
            if (Y >= 0 && Y < size && X >= 0 && X < size)
                map[Y, X] = 'X';
        }

        foreach (var (X, Y) in path)
        {
            if (Y >= 0 && Y < size && X >= 0 && X < size && map[Y, X] == '.')
                map[Y, X] = 'P';
        }

        if (edges != null)
        {
            foreach (var (X, Y) in edges)
            {
                if (Y >= 0 && Y < size && X >= 0 && X < size)
                {
                    map[Y, X] = 'O';
                }
            }
        }

        if (detourPoint.HasValue)
        {
            if (detourPoint.Value.Y >= 0 && detourPoint.Value.Y < size && detourPoint.Value.X >= 0 && detourPoint.Value.X < size)
                map[detourPoint.Value.Y, detourPoint.Value.X] = 'D';
        }

        for (int y = 0; y < size; y++)
        {
            for (int x = 0; x < size; x++)
            {
                Console.Write(map[y, x] + " ");
            }
            Console.WriteLine();
        }
    }
}