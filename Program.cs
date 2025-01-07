namespace bw_path_finding;

public class PathFinder
{
    /// <summary>
    /// Finds a path from the start point to the end point, stopping immediately upon encountering an obstacle.
    /// </summary>
    /// <param name="start">The starting point.</param>
    /// <param name="end">The destination point.</param>
    /// <param name="obstacles">A set of obstacle coordinates.</param>
    /// <returns>A tuple containing the path taken before interruption (if any),
    /// a list of valid detour points around the encountered obstacle,
    /// and the coordinates of the obstacle hit (if any).</returns>
    public (List<(int X, int Y)> Path, List<(int X, int Y)> ValidDetourPoints, (int X, int Y)? ObstacleHit) FindPathUntilObstacle((int X, int Y) start, (int X, int Y) end, HashSet<(int X, int Y)> obstacles)
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

            if (obstacles.Contains((currentX, currentY)))
            {
                // Obstacle encountered, find valid detour points.
                var connectedObstacles = GetConnectedObstacles((currentX, currentY), obstacles);
                var obstacleEdges = GetObstacleEdges(connectedObstacles);
                var validDetourPoints = FilterValidDetourPoints(start, obstacleEdges, obstacles);
                return (path, validDetourPoints, (currentX, currentY));
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

        return (path, [], null); // Reached the end without hitting an obstacle
    }

    /// <summary>
    /// Gets all obstacles connected to the starting obstacle tile using Breadth-First Search (BFS).
    /// </summary>
    /// <param name="startTile">The starting obstacle tile.</param>
    /// <param name="obstacles">A set of all obstacle coordinates.</param>
    /// <returns>A set of coordinates of connected obstacles.</returns>
    public HashSet<(int X, int Y)> GetConnectedObstacles((int X, int Y) startTile, HashSet<(int X, int Y)> obstacles)
    {
        if (!obstacles.Contains(startTile))
        {
            return [];
        }

        var connectedObstacles = new HashSet<(int X, int Y)>();
        var queue = new Queue<(int X, int Y)>();

        queue.Enqueue(startTile);
        connectedObstacles.Add(startTile);

        while (queue.Count > 0)
        {
            var current = queue.Dequeue();

            foreach (var neighbor in GetAdjacentTiles(current))
            {
                if (obstacles.Contains(neighbor) && !connectedObstacles.Contains(neighbor))
                {
                    queue.Enqueue(neighbor);
                    connectedObstacles.Add(neighbor);
                }
            }
        }
        return connectedObstacles;
    }

    private static IEnumerable<(int X, int Y)> GetAdjacentTiles((int X, int Y) tile)
    {
        yield return (tile.X - 1, tile.Y);
        yield return (tile.X + 1, tile.Y);
        yield return (tile.X, tile.Y - 1);
        yield return (tile.X, tile.Y + 1);
    }

    /// <summary>
    /// Gets the edge tiles of a group of obstacles. An edge tile is an obstacle tile with at least one non-obstacle neighbor.
    /// </summary>
    /// <param name="obstacles">A collection of obstacle coordinates.</param>
    /// <returns>A list of edge tiles among the obstacles.</returns>
    public List<(int X, int Y)> GetObstacleEdges(IEnumerable<(int X, int Y)> obstacles)
    {
        var obstacleSet = new HashSet<(int X, int Y)>(obstacles);
        var edges = new List<(int X, int Y)>();

        foreach (var tile in obstacles)
        {
            if (GetAdjacentTiles(tile).Any(neighbor => !obstacleSet.Contains(neighbor)))
            {
                edges.Add(tile);
            }
        }

        return edges.Distinct().ToList();
    }

    /// <summary>
    /// Filters a list of potential detour points to find those that are reachable from the start point without hitting obstacles.
    /// </summary>
    /// <param name="start">The starting point.</param>
    /// <param name="detourCandidates">A list of potential detour points.</param>
    /// <param name="obstacles">A hash set of obstacle coordinates for quick lookup.</param>
    /// <returns>A list of valid detour points reachable from the start point.</returns>
    public List<(int X, int Y)> FilterValidDetourPoints((int X, int Y) start, List<(int X, int Y)> detourCandidates, HashSet<(int X, int Y)> obstacles)
    {
        var validDetourPoints = new List<(int X, int Y)>();
        foreach (var candidate in detourCandidates)
        {
            if (IsReachable(start, candidate, obstacles))
            {
                validDetourPoints.Add(candidate);
            }
        }
        return validDetourPoints;
    }

    /// <summary>
    /// Finds the two outermost valid detour points from the perspective of the start point based on the angle.
    /// </summary>
    /// <param name="start">The starting point.</param>
    /// <param name="validDetourPoints">A list of valid detour points.</param>
    /// <returns>A tuple containing the leftmost and rightmost detour points, or null if fewer than two valid detour points exist.</returns>
    public ((int X, int Y), (int X, int Y))? FindOuterMostDetourPoints((int X, int Y) start, List<(int X, int Y)> validDetourPoints)
    {
        if (validDetourPoints == null || validDetourPoints.Count < 2)
        {
            return null;
        }

        var angles = validDetourPoints
            .Select(point => new { Point = point, Angle = Math.Atan2(point.Y - start.Y, point.X - start.X) })
            .OrderBy(item => item.Angle)
            .ToList();

        return (angles.First().Point, angles.Last().Point);
    }

    /// <summary>
    /// Selects the best detour point based on the distance to the goal.
    /// </summary>
    /// <param name="goal">The goal point.</param>
    /// <param name="outerMostDetourPoints">The outermost detour points.</param>
    /// <returns>The selected detour point.</returns>
    public (int X, int Y) SelectBestDetourPoint((int X, int Y) goal, ((int X, int Y), (int X, int Y))? outerMostDetourPoints)
    {
        if (outerMostDetourPoints == null)
        {
            return (-1, -1); // Indicate no valid detour points
        }

        double distanceToLeft = CalculateDistance(outerMostDetourPoints.Value.Item1, goal);
        double distanceToRight = CalculateDistance(outerMostDetourPoints.Value.Item2, goal);

        return distanceToLeft <= distanceToRight ? outerMostDetourPoints.Value.Item1 : outerMostDetourPoints.Value.Item2;
    }

    /// <summary>
    /// Finds an optimal detour point near the selected edge that is reachable from the start and closer to the goal.
    /// </summary>
    /// <param name="start">The starting point.</param>
    /// <param name="goal">The goal point.</param>
    /// <param name="selectedEdge">The selected edge point.</param>
    /// <param name="currentPath">The current path taken.</param>
    /// <param name="obstacles">The set of obstacles.</param>
    /// <returns>The optimal detour point, or null if none is found.</returns>
    public (int X, int Y)? FindOptimalDetourPoint((int X, int Y) start, (int X, int Y) goal, (int X, int Y) selectedEdge, List<(int X, int Y)> currentPath, HashSet<(int X, int Y)> obstacles)
    {
        var potentialDetours = GetAdjacentTiles(selectedEdge)
            .Where(tile => !obstacles.Contains(tile))
            .Concat(currentPath.Where(point => !obstacles.Contains(point)))
            .Distinct();

        (int X, int Y)? bestDetour = null;
        double minDistanceToGoal = double.MaxValue;

        foreach (var detour in potentialDetours)
        {
            if (IsReachable(start, detour, obstacles))
            {
                double distanceToGoal = CalculateDistance(detour, goal);
                if (distanceToGoal < minDistanceToGoal)
                {
                    minDistanceToGoal = distanceToGoal;
                    bestDetour = detour;
                }
            }
        }

        return bestDetour;
    }

    // Helper function to calculate Euclidean distance between two points.
    private static double CalculateDistance((int X, int Y) p1, (int X, int Y) p2)
    {
        int dx = p1.X - p2.X;
        int dy = p1.Y - p2.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    /// <summary>
    /// Checks if the end point is reachable from the start point without crossing any obstacles using a straight-line path (Bresenham's line algorithm).
    /// </summary>
    /// <param name="start">The starting point.</param>
    /// <param name="end">The destination point.</param>
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
        var start = (X: 0, Y: 0);
        var goal = (X: 13, Y: 14);

        var obstacles = new HashSet<(int X, int Y)>
        {
                (6, 7),     (7, 7),
        (5, 8), (6, 8),     (7, 8),     (8, 8),     (9, 8),
                (6, 9),     (7, 9),     (8, 9),     (9, 9),
                (6, 10),    (7, 10),    (8, 10),
                                        (8, 11)
        };

        var pathFinder = new PathFinder();
        var mapSize = 15;

        var (path, validDetourPoints, _) = pathFinder.FindPathUntilObstacle(start, goal, obstacles);

        var outerMostDetourPoints = pathFinder.FindOuterMostDetourPoints(start, validDetourPoints);
        var bestDetourPoint = pathFinder.SelectBestDetourPoint(goal, outerMostDetourPoints);

        (int X, int Y)? optimalDetourPoint = null;
        if (bestDetourPoint != (-1, -1))
        {
            optimalDetourPoint = pathFinder.FindOptimalDetourPoint(start, goal, bestDetourPoint, path, obstacles);
        }

        List<(int X, int Y)>? finalEdges = bestDetourPoint != (-1, -1) ? [bestDetourPoint] : null;

        DisplayMap(mapSize, start, goal, obstacles, path, finalEdges, optimalDetourPoint);
    }

    static void DisplayMap(int size, (int X, int Y) start, (int X, int Y) goal, HashSet<(int X, int Y)> obstacles, List<(int X, int Y)> path, List<(int X, int Y)>? edges, (int X, int Y)? detourPoint)
    {
        char[,] map = new char[size, size];

        // Initialize map with empty spaces
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