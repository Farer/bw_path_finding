namespace bw_path_finding;
public class PathFinder
{
    public (List<(int X, int Y)> path, (int X, int Y)? lastObstacle) FindNaturalPath((int X, int Y) start, (int X, int Y) end, List<(int X, int Y)> obstacles)
    {
        var path = new List<(int X, int Y)>();
        int x = start.X;
        int y = start.Y;

        int dx = Math.Abs(end.X - start.X);
        int dy = Math.Abs(end.Y - start.Y);

        int sx = start.X < end.X ? 1 : -1; // Direction of X increment
        int sy = start.Y < end.Y ? 1 : -1; // Direction of Y increment

        int err = dx - dy; // Initial error value

        while (true)
        {
            path.Add((x, y)); // Add current point to the path

            if (obstacles.Contains((x, y)))
            {
                return (path, (x, y)); // Stop if an obstacle is reached
            }

            if (x == end.X && y == end.Y)
            {
                break; // Stop if the destination point is reached
            }

            int e2 = 2 * err;

            if (e2 > -dy)
            {
                err -= dy;
                x += sx; // Move in X direction
            }
            else if (e2 < dx)
            {
                err += dx;
                y += sy; // Move in Y direction
            }
        }

        return (path, null); // Move to the destination without encountering obstacles
    }

    public List<(int X, int Y)> FilterValidEdges((int X, int Y) start, List<(int X, int Y)> edges, List<(int X, int Y)> obstacles)
    {
        var validEdges = new List<(int X, int Y)>();
        var obstacleSet = new HashSet<(int X, int Y)>(obstacles);

        foreach (var edge in edges)
        {
            // Test each edge without considering itself as an obstacle
            var tempObstacles = new HashSet<(int X, int Y)>(obstacleSet);
            tempObstacles.Remove(edge); // Temporarily exclude the edge from obstacles

            // Check if the path to the edge is clear
            var (_, lastObstacle) = FindNaturalPath(start, edge, [.. tempObstacles]);

            if (!lastObstacle.HasValue)
            {
                validEdges.Add(edge); // Add edge if it is reachable
            }
        }

        // Debugging: Output valid edges
        Console.WriteLine("Filtered Valid Edges:");
        foreach (var edge in validEdges)
        {
            Console.WriteLine($"Edge: {edge.X}:{edge.Y}");
        }

        return validEdges;
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

        var (path, lastObstacle) = pathFinder.FindNaturalPath(start, goal, obstacles);

        var edges = new List<(int X, int Y)>();

        if (lastObstacle.HasValue)
        {
            var connectedObstacles = pathFinder.GetConnectedObstacles(lastObstacle.Value, obstacles);
            edges = pathFinder.GetObstacleEdges(connectedObstacles);
            edges = pathFinder.FilterValidEdges(start, edges, obstacles);
        }

        DisplayMap(size, start, goal, obstacles, path, edges);
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

        for (int y = 0; y < size; y++)
        {
            for (int x = 0; x < size; x++)
                Console.Write(map[y, x] + " ");
            Console.WriteLine();
        }
    }
}
