using System;
using System.Collections.Generic;
using System.Linq;

public class AnyaPathfinding
{
    private int[,] grid;
    private int rows;
    private int cols;

    public AnyaPathfinding(int[,] grid)
    {
        this.grid = grid;
        this.rows = grid.GetLength(0);
        this.cols = grid.GetLength(1);
    }

    public List<(int x, int y)>? FindPath((int x, int y) start, (int x, int y) goal)
    {
        var openList = new SortedSet<(double, double, (int, int), (int, int), List<(int, int)>)>(
            Comparer<(double, double, (int, int), (int, int), List<(int, int)>)>.Create((a, b) => a.Item1.CompareTo(b.Item1))
        );
        openList.Add((0, 0, start, start, new List<(int, int)> { start }));

        var closedList = new HashSet<(int, int)>();

        while (openList.Count > 0)
        {
            var current = openList.First();
            openList.Remove(current);

            if (current.Item4 == goal) // current.p
                return current.Item5; // current.path

            if (closedList.Contains(current.Item4)) // current.p
                continue;

            closedList.Add(current.Item4);

            foreach (var successor in GetSuccessors(current.Item4, current.Item3)) // current.p, current.r
            {
                if (closedList.Contains(successor.p))
                    continue;

                double g = current.Item2 + Distance(current.Item4, successor.p); // current.g, current.p
                double h = Distance(successor.p, goal);
                double f = g + h;

                var newPath = new List<(int, int)>(current.Item5) { successor.p }; // current.path
                openList.Add((f, g, successor.r, successor.p, newPath));
            }
        }

        return null; // No path found
    }

    private IEnumerable<((int x, int y) r, (int x, int y) p)> GetSuccessors((int x, int y) current, (int x, int y) root)
    {
        var directions = new (int dx, int dy)[]
        {
            (-1, 0), (1, 0), (0, -1), (0, 1), // Cardinal
            // (-1, -1), (-1, 1), (1, -1), (1, 1) // Diagonal
        };

        foreach (var (dx, dy) in directions)
        {
            var next = (x: current.x + dx, y: current.y + dy);

            if (IsTraversable(next))
            {
                yield return (root, next);
            }
        }
    }

    private bool IsTraversable((int x, int y) point)
    {
        return point.x >= 0 && point.x < rows && point.y >= 0 && point.y < cols && grid[point.x, point.y] == 0;
    }

    private double Distance((int x, int y) a, (int x, int y) b)
    {
        return Math.Sqrt(Math.Pow(a.x - b.x, 2) + Math.Pow(a.y - b.y, 2));
    }

    public static void Main()
    {
        int[,] grid = {
            { 0, 0, 0, 1, 0 },
            { 0, 1, 0, 1, 0 },
            { 0, 1, 0, 0, 0 },
            { 0, 0, 0, 1, 0 },
            { 1, 1, 0, 0, 0 }
        };

        var anya = new AnyaPathfinding(grid);
        var start = (x: 0, y: 0);
        var goal = (x: 4, y: 4);

        var path = anya.FindPath(start, goal);

        if (path != null)
        {
            Console.WriteLine("Path found:");
            foreach (var step in path)
            {
                Console.WriteLine($"({step.x}, {step.y})");
            }
        }
        else
        {
            Console.WriteLine("No path found.");
        }
    }
}
