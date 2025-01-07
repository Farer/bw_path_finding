﻿namespace bw_path_finding;

public class PathFinder
{
    /// <summary>
    /// Finds a natural path from the start point to the end point, stopping immediately upon encountering an obstacle.
    /// </summary>
    /// <param name="start">The starting point.</param>
    /// <param name="end">The destination point.</param>
    /// <param name="obstacles">A list of obstacle coordinates.</param>
    /// <returns>A tuple containing the path taken before interruption (if any), a list of valid edges around the encountered obstacle, and the coordinates of the obstacle hit.</returns>
    public (List<(int X, int Y)> path, List<(int X, int Y)> validEdges) FindNaturalPath((int X, int Y) start, (int X, int Y) end, HashSet<(int X, int Y)> obstacles)
    {
        var path = new List<(int X, int Y)>();
        int x = start.X;
        int y = start.Y;

        int dx = Math.Abs(end.X - start.X);
        int dy = Math.Abs(end.Y - start.Y);

        int sx = start.X < end.X ? 1 : -1;
        int sy = start.Y < end.Y ? 1 : -1;

        int err = dx - dy;

        HashSet<(int X, int Y)> obstacleSet = obstacles.ToHashSet();

        while (true)
        {
            path.Add((x, y));

            if (obstacleSet.Contains((x, y)))
            {
                // Find all connected obstacles and their valid edges when an obstacle is encountered.
                var connectedObstacles = GetConnectedObstacles((x, y), obstacles);
                var obstacleEdges = GetObstacleEdges(connectedObstacles);
                var validEdges = FilterValidEdgesFromObstacle(start, obstacleEdges, obstacleSet, (x, y)); // Pass the obstacle hit point
                return (path, validEdges);
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

        return (path, []);
    }

    /// <summary>
    /// Gets all obstacles connected to the starting obstacle tile using Breadth-First Search (BFS).
    /// </summary>
    /// <param name="startTile">The starting obstacle tile.</param>
    /// <param name="obstacles">A list of all obstacle coordinates.</param>
    /// <returns>A list of coordinates of connected obstacles.</returns>
    public List<(int X, int Y)> GetConnectedObstacles((int X, int Y) startTile, HashSet<(int X, int Y)> obstacles)
    {
        var connected = new List<(int X, int Y)>();
        var visited = new HashSet<(int X, int Y)>();
        var queue = new Queue<(int X, int Y)>();
        var obstacleSet = new HashSet<(int X, int Y)>(obstacles);

        if (!obstacleSet.Contains(startTile)) return connected;

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
                if (obstacleSet.Contains(neighbor) && !visited.Contains(neighbor))
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

    /// <summary>
    /// Gets the edge tiles of a group of obstacles. An edge tile is an obstacle tile with at least one non-obstacle neighbor.
    /// </summary>
    /// <param name="obstacles">A list of obstacle coordinates.</param>
    /// <returns>A list of edge tiles among the obstacles.</returns>
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

            // Check if any neighbor is not an obstacle
            if (neighbors.Any(n => !obstacleSet.Contains(n)))
            {
                edges.Add(tile);
            }
        }

        Console.WriteLine("Detected Obstacle Edges:");
        foreach (var edge in edges)
        {
            Console.WriteLine($"Edge: {edge.X}:{edge.Y}");
        }

        return edges.Distinct().ToList(); // Remove duplicates
    }

    /// <summary>
    /// Filters a list of potential edge tiles to find those that are reachable from the start point without hitting obstacles,
    /// prioritizing edges that move towards the start point from the obstacle.
    /// </summary>
    /// <param name="start">The starting point.</param>
    /// <param name="edges">A list of potential edge tiles.</param>
    /// <param name="obstacles">A hash set of obstacle coordinates for quick lookup.</param>
    /// <param name="obstacleHit">The coordinates of the obstacle that was hit.</param>
    /// <returns>A list of valid edge tiles reachable from the start point.</returns>
    public List<(int X, int Y)> FilterValidEdgesFromObstacle((int X, int Y) start, List<(int X, int Y)> edges, HashSet<(int X, int Y)> obstacles, (int X, int Y) obstacleHit)
    {
        var validEdges = new List<(int X, int Y)>();

        // Sort edges based on their "towards the start" direction
        var sortedEdges = edges.OrderBy(edge =>
        {
            // Prioritize edges closer to the start point
            var diffXStart = Math.Abs(edge.X - start.X);
            var diffYStart = Math.Abs(edge.Y - start.Y);

            return diffXStart + diffYStart;
        }).ToList();

        Console.WriteLine("Sorted Edges:");
        foreach (var edge in sortedEdges)
        {
            Console.WriteLine($"Edge: {edge.X}:{edge.Y}");
        }

        foreach (var edge in sortedEdges)
        {
            if (IsReachable(start, edge, obstacles))
            {
                validEdges.Add(edge);
            }
        }
        Console.WriteLine("Filtered Valid Edges:");
        foreach (var edge in validEdges)
        {
            Console.WriteLine($"Edge: {edge.X}:{edge.Y}");
        }
        return validEdges;
    }

    /// <summary>
    /// Finds the two outermost valid edge coordinates from the perspective of the start point.
    /// </summary>
    /// <param name="start">The starting point.</param>
    /// <param name="validEdges">A list of valid edge coordinates.</param>
    /// <returns>A tuple containing the leftmost and rightmost edge coordinates, or null if fewer than two valid edges exist.</returns>
    public ((int X, int Y), (int X, int Y))? FindOuterMostEdges((int X, int Y) start, List<(int X, int Y)> validEdges)
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

    public (int X, int Y) SelectBestDetourPoint((int X, int Y) goal, ((int X, int Y), (int X, int Y))? outerMostEdges)
    {
        if (outerMostEdges == null)
        {
            // Handle cases where there are no valid outer edges.
            return (-1, -1);
        }

        var leftmost = outerMostEdges.Value.Item1;
        var rightmost = outerMostEdges.Value.Item2;

        // Calculate distances from candidates to the goal.
        double distanceToLeft = CalculateDistance(leftmost, goal);
        double distanceToRight = CalculateDistance(rightmost, goal);

        if (distanceToLeft <= distanceToRight)
        {
            Console.WriteLine($"Selected Detour Point: {leftmost.X}:{leftmost.Y} (Leftmost)");
            return leftmost;
        }
        else
        {
            Console.WriteLine($"Selected Detour Point: {rightmost.X}:{rightmost.Y} (Rightmost)");
            return rightmost;
        }
    }

    public (int X, int Y)? FindOptimalDetourPoint((int X, int Y) start, (int X, int Y) goal, (int X, int Y) selectedEdge, List<(int X, int Y)> currentPath, HashSet<(int X, int Y)> obstacles)
    {
        var obstacleSet = new HashSet<(int X, int Y)>(obstacles);
        var potentialDetours = new List<(int X, int Y)>();

        // Explore 8 neighboring tiles (including diagonals).
        var neighbors = new List<(int X, int Y)>
        {
            (selectedEdge.X - 1, selectedEdge.Y - 1),
            (selectedEdge.X - 1, selectedEdge.Y),
            (selectedEdge.X - 1, selectedEdge.Y + 1),
            (selectedEdge.X, selectedEdge.Y - 1),
            (selectedEdge.X, selectedEdge.Y + 1),
            (selectedEdge.X + 1, selectedEdge.Y - 1),
            (selectedEdge.X + 1, selectedEdge.Y),
            (selectedEdge.X + 1, selectedEdge.Y + 1)
        };

        // Include points from the current path as candidates.
        foreach (var pathPoint in currentPath)
        {
            if (!potentialDetours.Contains(pathPoint) && !obstacleSet.Contains(pathPoint))
            {
                potentialDetours.Add(pathPoint);
            }
        }

        foreach (var neighbor in neighbors)
        {
            if (!obstacleSet.Contains(neighbor)) // Check if not an obstacle.
            {
                potentialDetours.Add(neighbor);
            }
        }

        (int X, int Y)? bestDetour = null;
        double minDistanceToGoal = double.MaxValue;

        foreach (var detour in potentialDetours.Distinct())
        {
            if (IsReachable(start, detour, obstacleSet))
            {
                double distanceToGoal = CalculateDistance(detour, goal);
                if (distanceToGoal < minDistanceToGoal)
                {
                    minDistanceToGoal = distanceToGoal;
                    bestDetour = detour;
                }
            }
        }

        if (bestDetour != null)
        {
            Console.WriteLine($"Optimal Detour Point: {bestDetour.Value.X}:{bestDetour.Value.Y}");
        }
        else
        {
            Console.WriteLine("No optimal detour point found.");
        }

        return bestDetour;
    }

    // Helper function to calculate Euclidean distance between two points.
    private double CalculateDistance((int X, int Y) p1, (int X, int Y) p2)
    {
        int dx = p1.X - p2.X;
        int dy = p1.Y - p2.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    /// <summary>
    /// Checks if the end point is reachable from the start point without crossing any obstacles using a straight-line path.
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
            // Safety break to prevent infinite loops in unforeseen scenarios
            if (Math.Abs(x - start.X) > Math.Abs(end.X - start.X) || Math.Abs(y - start.Y) > Math.Abs(end.Y - start.Y))
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

        var size = 15;

        var (path, validEdges) = pathFinder.FindNaturalPath(start, goal, obstacles);

        var OuterMostEdges = pathFinder.FindOuterMostEdges(start, validEdges);
        var finalLocation = pathFinder.SelectBestDetourPoint(goal, OuterMostEdges);
        if(finalLocation == (-1, -1))
        {
            DisplayMap(size, start, goal, obstacles, path, null, null);
            return;
        }
        else {
            var optimalDetourPoint = pathFinder.FindOptimalDetourPoint(start, goal, finalLocation, path, obstacles);

            List<(int X, int Y)> finalEdges = new List<(int X, int Y)>();
            finalEdges.Add(finalLocation);
            if (optimalDetourPoint.HasValue)
            {
                finalEdges.Add(optimalDetourPoint.Value);
            }

            DisplayMap(size, start, goal, obstacles, path, finalEdges, optimalDetourPoint);
        }
    }

    static void DisplayMap(int size, (int X, int Y) start, (int X, int Y) goal, HashSet<(int X, int Y)> obstacles, List<(int X, int Y)> path, List<(int X, int Y)>? edges, (int X, int Y)? detourPoint)
    {
        char[,] map = new char[size, size];

        for (int i = 0; i < size; i++)
            for (int j = 0; j < size; j++)
                map[i, j] = '.';

        map[start.Y, start.X] = 'S';
        map[goal.Y, goal.X] = 'E';

        foreach (var obstacle in obstacles)
            map[obstacle.Y, obstacle.X] = 'X';

        foreach (var point in path)
            if (map[point.Y, point.X] == '.')
                map[point.Y, point.X] = 'P';

        if(edges != null) {
            foreach (var point in edges)
                if (map[point.Y, point.X] == 'X')
                    map[point.Y, point.X] = 'O';
                else if (map[point.Y, point.X] == '.')
                    map[point.Y, point.X] = 'O';
        }

        if (detourPoint.HasValue)
        {
            map[detourPoint.Value.Y, detourPoint.Value.X] = 'D';
        }

        for (int y = 0; y < size; y++)
        {
            for (int x = 0; x < size; x++)
                Console.Write(map[y, x] + " ");
            Console.WriteLine();
        }
    }
}
