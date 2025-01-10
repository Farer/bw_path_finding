using System.Collections.Concurrent;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace bw_path_finding;

public class PathFinder
{
    // 캐싱을 위한 Dictionary (인스턴스 별로 관리)
    // private Dictionary<((int X, int Y) start, (int X, int Y) end), bool> _reachableCache = new Dictionary<((int X, int Y) start, (int X, int Y) end), bool>();
    private ConcurrentDictionary<((int X, int Y) start, (int X, int Y) end), bool> _reachableCache = new();

    public HashSet<(int X, int Y)> GetAllValidEdges((int X, int Y) start, (int X, int Y) targetEdge, int sight, HashSet<(int X, int Y)> obstacles, int mapSize)
    {
        var validEdges = new HashSet<(int X, int Y)>();
        var queue = new Queue<((int X, int Y) point, int distance)>();
        var visited = new HashSet<(int X, int Y)>();

        queue.Enqueue((targetEdge, 0));
        visited.Add(targetEdge);

        while (queue.Count > 0)
        {
            var (currentPoint, distance) = queue.Dequeue();

            if (distance > sight)
            {
                continue;
            }

            // Check if the current point is a valid edge
            var nearTiles = GetNearTiles(currentPoint);
            if (nearTiles.Any(tile => !obstacles.Contains(tile) && RaycastDDA(start, currentPoint, mapSize, obstacles, mapSize) != null))
            {
                validEdges.Add(currentPoint);
            }

            // Explore neighbors
            foreach (var neighbor in GetAdjacentTiles(currentPoint))
            {

                if (neighbor.X >= 0 && neighbor.X < mapSize && neighbor.Y >= 0 && neighbor.Y < mapSize &&
                    obstacles.Contains(neighbor) && !visited.Contains(neighbor))
                {
                    visited.Add(neighbor);
                    queue.Enqueue((neighbor, distance + 1));
                }
            }
        }

        return validEdges;
    }
    /// <summary>
    /// Implements raycasting on a 2D grid using the Digital Differential Analyzer (DDA) algorithm for accurate line tracing.
    /// </summary>
    /// <param name="start">The starting coordinates of the ray.</param>
    /// <param name="target">The target coordinates the ray is heading towards.</param>
    /// <param name="maxDistance">The maximum travel distance of the ray.</param>
    /// <param name="obstacles">A set of obstacle coordinates.</param>
    /// <param name="mapSize">The size of the map.</param>
    /// <returns>The coordinates of the hit obstacle or null.</returns>
    public static (int X, int Y)? RaycastDDA(
        (int X, int Y) start,
        (int X, int Y) target,
        int maxDistance,
        HashSet<(int X, int Y)> obstacles,
        int mapSize
    )
    {
        int startX = start.X;
        int startY = start.Y;
        int targetX = target.X;
        int targetY = target.Y;

        int dx = targetX - startX;
        int dy = targetY - startY;

        if (dx == 0 && dy == 0) return null;

        int currentX = startX;
        int currentY = startY;

        int stepX = Math.Sign(dx);
        int stepY = Math.Sign(dy);

        float rayDirX = dx == 0 ? 0 : 1f / MathF.Abs(dx);
        float rayDirY = dy == 0 ? 0 : 1f / MathF.Abs(dy);

        float tMaxX = (stepX > 0 ? (float)(1 - (startX % 1)) : (float)(startX % 1)) * rayDirX;
        float tMaxY = (stepY > 0 ? (float)(1 - (startY % 1)) : (float)(startY % 1)) * rayDirY;

        float tDeltaX = rayDirX;
        float tDeltaY = rayDirY;

        int steps = 0;

        while (steps < maxDistance)
        {
            if (currentX < 0 || currentX >= mapSize || currentY < 0 || currentY >= mapSize)
            {
                return null; // Out of bounds
            }

            if (obstacles.Contains((currentX, currentY)))
            {
                return (currentX, currentY); // Hit obstacle
            }

            if (tMaxX < tMaxY)
            {
                tMaxX += tDeltaX;
                currentX += stepX;
            }
            else
            {
                tMaxY += tDeltaY;
                currentY += stepY;
            }

            steps++;
        }

        return null; // No obstacle hit within max distance
    }

    /// <summary>
    /// Checks if the end point is reachable from the start point without crossing any obstacles using a straight-line path (Bresenham's line algorithm).
    /// </summary>
    public bool IsReachable((int X, int Y) start, (int X, int Y) end, HashSet<(int X, int Y)> obstacles)
    {
        var cacheKey = (start, end);
        return _reachableCache.GetOrAdd(cacheKey, _ =>
        {
            int x = start.X;
            int y = start.Y;
            int dx = Math.Abs(end.X - start.X);
            int dy = Math.Abs(end.Y - start.Y);
            int sx = start.X < end.X ? 1 : -1;
            int sy = start.Y < end.Y ? 1 : -1;
            int err = dx - dy;
            bool isReachable = true;

            while (true)
            {
                if (obstacles.Contains((x, y)) && (x, y) != end)
                {
                    isReachable = false;
                    break;
                }

                if (x == end.X && y == end.Y)
                {
                    break;
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
            }
            return isReachable;
        });
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

    public static IEnumerable<(int X, int Y)> GetNearTiles((int X, int Y) tile)
    {
        yield return (tile.X - 1, tile.Y);
        yield return (tile.X + 1, tile.Y);
        yield return (tile.X, tile.Y - 1);
        yield return (tile.X, tile.Y + 1);
    }

    #region 나머지 코드는 이전과 동일합니다.
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
            int mapSize = 15,
            int sight = 30
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
        Console.WriteLine("isReverse: " + isReverse);
        var validEdgePoints = new List<(int X, int Y)>();
        var visitedEdges = new HashSet<(int X, int Y)>();
        var queue = new Queue<(int X, int Y)>();

        // Add the exterior tiles of the first encountered obstacle to the queue.
        var adjacentTiles = GetAdjacentTiles(obstacleHit);
        foreach (var neighbor in adjacentTiles)
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
        var currentEdgePoint = startEdge;

        while (true)
        {
            var isReachable = IsReachable(start, currentEdgePoint, obstacles);
            if (isReachable && !validEdgePoints.Contains(currentEdgePoint))
            {
                validEdgePoints.Add(currentEdgePoint);
                Console.WriteLine($"Edge point: {currentEdgePoint}");
            }

            bool foundNextEdge = false;
            foreach (var (dx, dy) in possibleDirections)
            {
                var nextEdgeCandidate = (currentEdgePoint.X + dx, currentEdgePoint.Y + dy);
                if (obstacles.Contains(nextEdgeCandidate) && visitedEdges.Add(nextEdgeCandidate))
                {
                    currentEdgePoint = nextEdgeCandidate;
                    foundNextEdge = true;
                    break;
                }
            }

            if (!foundNextEdge) break; // No more edges to explore

            // If the current edge is not reachable from the start, the previous one might be a valid detour point
            var isCurrentEdgeReachable = IsReachable(start, currentEdgePoint, obstacles);
            if (!isCurrentEdgeReachable)
            {
                break;
            }

            if (currentEdgePoint == startEdge && validEdgePoints.Count > 0) break; // Back to the starting edge
        }

        if (validEdgePoints.Count == 0)
        {
            validEdgePoints.Add(obstacleHit);
        }
        return validEdgePoints;
    }

    private bool IsWithinMapBounds(int x, int y, int mapWidth, int mapHeight)
    {
        return x >= 0 && x < mapWidth && y >= 0 && y < mapHeight;
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
        if (!isReverse)
        {
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
        else
        {
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

    public List<(int X, int Y)> GetConnectedObstacles((int X, int Y) startTile, List<(int X, int Y)> obstacles)
    {
        var connected = new List<(int X, int Y)>();
        var visited = new HashSet<(int X, int Y)>();
        var queue = new Queue<(int X, int Y)>();

        if (!obstacles.Contains(startTile)) return connected;

        queue.Enqueue(startTile);
        visited.Add(startTile);

        while (queue.Count > 0)
        {
            var current = queue.Dequeue();
            connected.Add(current);

            var neighbors = new List<(int X, int Y)>
            {
                (current.X - 1, current.Y),
                (current.X + 1, current.Y),
                (current.X, current.Y - 1),
                (current.X, current.Y + 1)
            };

            foreach (var neighbor in neighbors)
            {
                if (obstacles.Contains(neighbor) && !visited.Contains(neighbor))
                {
                    queue.Enqueue(neighbor);
                    visited.Add(neighbor);
                }
            }
        }

        Console.WriteLine("Connected Obstacles:");
        foreach (var tile in connected)
        {
            Console.WriteLine($"Obstacle: {tile.X}:{tile.Y}");
        }

        return connected;
    }

    public List<(int X, int Y)> GetObstacleEdges(List<(int X, int Y)> obstacles)
    {
        var edges = new List<(int X, int Y)>();
        var obstacleSet = new HashSet<(int X, int Y)>(obstacles);

        foreach (var tile in obstacles)
        {
            var neighbors = new List<(int X, int Y)>
            {
                (tile.X, tile.Y - 1),
                (tile.X, tile.Y + 1),
                (tile.X - 1, tile.Y),
                (tile.X + 1, tile.Y)
            };

            if (neighbors.Exists(n => !obstacleSet.Contains(n)))
            {
                edges.Add(tile);
            }
        }

        Console.WriteLine("Detected Obstacle Edges:");
        foreach (var edge in edges)
        {
            Console.WriteLine($"Edge: {edge.X}:{edge.Y}");
        }

        return edges;
    }

    public ((int X, int Y), (int X, int Y))? FindOuterMostEdges((int X, int Y) start, HashSet<(int X, int Y)> validEdges)
    {
        if (validEdges == null || validEdges.Count < 2)
        {
            return null; // Not enough valid edges to find outermost
        }

        // Calculate the angle from the start point to each valid edge.
        var angles = validEdges.Select(edge => new
        {
            Edge = edge,
            Angle = Math.Atan2(edge.Y - start.Y, edge.X - start.X)
        }).ToList();

        // Sort the angles in ascending order.
        angles.Sort((a, b) => a.Angle.CompareTo(b.Angle));

        // The first element in the sorted list will have the smallest angle (leftmost),
        // and the last element will have the largest angle (rightmost).
        var leftmost = angles.First().Edge;
        var rightmost = angles.Last().Edge;

        Console.WriteLine($"Leftmost Edge: {leftmost.X}:{leftmost.Y}");
        Console.WriteLine($"Rightmost Edge: {rightmost.X}:{rightmost.Y}");

        return (leftmost, rightmost);
    }

    // public (int X, int Y) DetermineFinalTargetEdge(((int X, int Y), (int X, int Y)) outerMostEdges, int mapSize)
    // {
    //     var leftNearTiles = GetNearTiles(outerMostEdges.Item1);
    //     bool isLeftOutOfMap = false;
    //     foreach (var tile in leftNearTiles)
    //     {
    //         // Console.WriteLine($"Left near tile: {tile}");
    //         if (tile.X < 0 || tile.X >= mapSize || tile.Y < 0 || tile.Y >= mapSize)
    //         {
    //             isLeftOutOfMap = true;
    //             Console.WriteLine($"out of Map: {tile}");
    //             break;
    //         }
    //     }

    //     var rightNearTiles = GetNearTiles(outerMostEdges.Item2);
    //     bool isRightOutOfMap = false;
    //     foreach (var tile in rightNearTiles)
    //     {
    //         Console.WriteLine($"Right near tile: {tile}");
    //         if (tile.X < 0 || tile.X >= mapSize || tile.Y < 0 || tile.Y >= mapSize)
    //         {
    //             isRightOutOfMap = true;
    //             Console.WriteLine($"out of Map: {tile}");
    //             break;
    //         }
    //     }

    //     if (isLeftOutOfMap && isRightOutOfMap)
    //     {
    //         Console.WriteLine("Both edges are near the border");
    //         return (-1, -1);
    //     }
    //     else if (isLeftOutOfMap)
    //     {
    //         Console.WriteLine("Left edge is near the border");
    //         return outerMostEdges.Item2;
    //     }
    //     else if (isRightOutOfMap)
    //     {
    //         Console.WriteLine("Right edge is near the border");
    //         return outerMostEdges.Item1;
    //     }
    //     else
    //     {
    //         Console.WriteLine("Both edges are not near the border");

    //     }
    // }
    #endregion
}

class Program
{
    static void Main()
    {
        var pathFinder = new PathFinder();
        var mapSize = 15;
        mapSize = 9999;
        var start = (X: -1, Y: -1); var goal = (X: -1, Y: -1);
        start = (X: 0, Y: 0); goal = (X: 6, Y: 14);

        var obstacles = new HashSet<(int X, int Y)>
        {
                                                            (5, 4),
                                                            (5, 5),
                (1, 6),     (2, 6),                         (5, 6),
        (0, 7), (1, 7),     (2, 7),     (3, 7),     (4, 7),
                (1, 8),     (2, 8),     (3, 8),     (4, 8),
                (1, 9),     (2, 9),     (3, 9),
                                        (3, 10)
        };

        (int X, int Y)? raycastResult = (X: -1, Y: -1);
        List<(int?, int?)> raycastResults = []; // 결과를 저장할 리스트
        Stopwatch stopwatch = new();
        stopwatch.Start();
        for(var i=0; i<mapSize; i++) {
            goal = (X: 0, Y: i);
            raycastResult = PathFinder.RaycastDDA(start, goal, mapSize, obstacles, mapSize);
            raycastResults.Add((raycastResult?.X, raycastResult?.Y)); // 결과를 리스트에 추가하여 사용
        }
        for(var i=0; i<mapSize; i++) {
            goal = (X: i, Y: 0);
            raycastResult = PathFinder.RaycastDDA(start, goal, mapSize, obstacles, mapSize);
            raycastResults.Add((raycastResult?.X, raycastResult?.Y)); // 결과를 리스트에 추가하여 사용
        }
        stopwatch.Stop();
        Console.WriteLine($"Raycast Elapsed time: {stopwatch.ElapsedMilliseconds}ms");

        bool isReachable = false;
        List<bool> reahableResults = []; // 결과를 저장할 리스트
        stopwatch.Restart();
        var requests = new List<((int X, int Y) start, (int X, int Y) goal)>();
        for(var i=0; i<mapSize; i++) {
            goal = (X: 0, Y: i);
            requests.Add((start, goal));
        }
        Parallel.ForEach(requests, request =>
        {
            bool isReachable = pathFinder.IsReachable(request.start, request.goal, obstacles);
            // isReachable 결과를 처리 (예: 로깅, 다른 컬렉션에 저장 등)
            // Console.WriteLine($"Start: {request.start}, Goal: {request.goal}, Reachable: {isReachable}");
        });
        stopwatch.Stop();
        Console.WriteLine($"IsReachable Elapsed time: {stopwatch.ElapsedMilliseconds}ms");
        Environment.Exit(0);


        var allValidEdges = pathFinder.GetAllValidEdges(start, (0, 7), 15, obstacles, mapSize);
        foreach (var edge in allValidEdges)
        {
            Console.WriteLine($"Valid Edge: {edge}");
        }

        var outerMostEdges = pathFinder.FindOuterMostEdges(start, allValidEdges);
        Console.WriteLine($"leftmost: {outerMostEdges?.Item1}, rightmost: {outerMostEdges?.Item2}");

        Environment.Exit(0);

        // int sight = 5; // 시야 거리 설정

        // var (path, validSearchEdges) = pathFinder.FindPathUntilObstacle(start, goal, obstacles, false, mapSize, sight);

        // if (validSearchEdges.Count == 0)
        // {
        //     DisplayMap(mapSize, start, goal, obstacles, path, null, null);
        // }
        // else if (validSearchEdges.Count > 0)
        // {
        //     bool isReverse = false;
        //     var (X, Y) = validSearchEdges.Last();
        //     if (X <= 0 || X >= mapSize - 1)
        //     {
        //         isReverse = true;
        //         (path, validSearchEdges) = pathFinder.FindPathUntilObstacle(start, goal, obstacles, true, mapSize, sight);
        //         if (validSearchEdges.Count == 0)
        //         {
        //             DisplayMap(mapSize, start, goal, obstacles, path, null, null);
        //         }
        //     }
        //     var bestEdgePoint = validSearchEdges.Last();
        //     var allValidEdges = pathFinder.GetAllValidEdges(start, bestEdgePoint, sight, obstacles, mapSize);
        //     (int X, int Y)? optimalDetourPoint = null;
        //     if (bestEdgePoint != (-1, -1))
        //     {
        //         optimalDetourPoint = pathFinder.FindOptimalDetourPoint(start, goal, bestEdgePoint, path, obstacles, isReverse);
        //         Console.WriteLine($"Optimal detour point: {optimalDetourPoint}");
        //     }


        //     DisplayMap(mapSize, start, goal, obstacles, path, allValidEdges, optimalDetourPoint);
        // }
    }

    static void DisplayMap(int size, (int X, int Y) start, (int X, int Y) goal, HashSet<(int X, int Y)> obstacles, List<(int X, int Y)> path, HashSet<(int X, int Y)>? edges, (int X, int Y)? detourPoint)
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