namespace bw_path_finding;

public class PathFinder
{
    /// <summary>
    /// Finds a natural path from the start point to the end point, stopping immediately upon encountering an obstacle.
    /// </summary>
    /// <param name="start">The starting point.</param>
    /// <param name="end">The destination point.</param>
    /// <param name="obstacles">A list of obstacle coordinates.</param>
    /// <returns>A tuple containing the path taken before interruption (if any) and a list of valid edges around the encountered obstacle.</returns>
    public (List<(int X, int Y)> path, List<(int X, int Y)> validEdges) FindNaturalPath((int X, int Y) start, (int X, int Y) end, List<(int X, int Y)> obstacles)
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
                // When an obstacle is encountered, find all connected obstacles and their valid edges.
                var connectedObstacles = GetConnectedObstacles((x, y), obstacles);
                var obstacleEdges = GetObstacleEdges(connectedObstacles);
                var validEdges = FilterValidEdgesFromObstacle(start, obstacleEdges, obstacleSet);
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
    /// Gets all obstacles connected to the starting tile.
    /// </summary>
    /// <param name="startTile">The starting obstacle tile.</param>
    /// <param name="obstacles">A list of all obstacle coordinates.</param>
    /// <returns>A list of coordinates of connected obstacles.</returns>
    public List<(int X, int Y)> GetConnectedObstacles((int X, int Y) startTile, List<(int X, int Y)> obstacles)
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
    /// Filters a list of potential edge tiles to find those that are reachable from the start point without hitting obstacles.
    /// </summary>
    /// <param name="start">The starting point.</param>
    /// <param name="edges">A list of potential edge tiles.</param>
    /// <param name="obstacles">A hash set of obstacle coordinates for quick lookup.</param>
    /// <returns>A list of valid edge tiles reachable from the start point.</returns>
    public List<(int X, int Y)> FilterValidEdgesFromObstacle((int X, int Y) start, List<(int X, int Y)> edges, HashSet<(int X, int Y)> obstacles)
    {
        var validEdges = new List<(int X, int Y)>();

        foreach (var edge in edges)
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
    /// Checks if the end point is reachable from the start point without crossing any obstacles.
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
        var goal = (X: 10, Y: 13);

        var obstacles = new List<(int X, int Y)>
        {
                    (4, 7),     (5, 7),
            (3, 8), (4, 8),     (5, 8),     (6, 8),     (7, 8),
                    (4, 9),     (5, 9),     (6, 9),     (7, 9),
                    (4, 10),    (5, 10),    (6, 10),
                                            (6, 11)
        };

        var pathFinder = new PathFinder();

        var size = 15;

        var (path, validEdges) = pathFinder.FindNaturalPath(start, goal, obstacles);

        DisplayMap(size, start, goal, obstacles, path, validEdges);
    }

    static void DisplayMap(int size, (int X, int Y) start, (int X, int Y) goal, List<(int X, int Y)> obstacles, List<(int X, int Y)> path, List<(int X, int Y)> edges)
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

        foreach (var point in edges)
            if (map[point.Y, point.X] == 'X')
                map[point.Y, point.X] = 'O';
            else if (map[point.Y, point.X] == '.')
                map[point.Y, point.X] = 'O';

        for (int y = 0; y < size; y++)
        {
            for (int x = 0; x < size; x++)
                Console.Write(map[y, x] + " ");
            Console.WriteLine();
        }
    }
}