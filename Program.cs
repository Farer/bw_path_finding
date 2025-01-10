namespace bw_path_finding;
// Represents a point in 2D space
public struct Point
{
    public double X { get; set; }
    public double Y { get; set; }

    public Point(double x, double y)
    {
        X = x;
        Y = y;
    }

    public static double Distance(Point p1, Point p2)
    {
        return Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
    }
}

public class TangentBugAlgorithm
{
    private Point _robotPosition;
    private Point _goalPosition;
    private List<List<Point>> _obstacles;
    private double _sensorRange;
    private double _stepSize;

    private enum State { MovingToGoal, FollowingBoundary };
    private State _currentState;
    private List<Point> _currentBoundary = new List<Point>();
    private double _dMin = double.MaxValue;
    private double _dLeave = double.MaxValue;
    private Point _hitPoint;
    private bool _followClockwise = true; // Default to clockwise boundary following

    public TangentBugAlgorithm(Point start, Point goal, List<List<Point>> obstacles, double sensorRange = 10, double stepSize = 0.1)
    {
        _robotPosition = start;
        _goalPosition = goal;
        _obstacles = obstacles;
        _sensorRange = sensorRange;
        _stepSize = stepSize;
        _currentState = State.MovingToGoal;
    }

    public List<Point> FindPath()
    {
        var path = new List<Point> { _robotPosition };

        while (Point.Distance(_robotPosition, _goalPosition) > _stepSize) // Add a tolerance
        {
            switch (_currentState)
            {
                case State.MovingToGoal:
                    MoveTowardsGoal(path);
                    break;
                case State.FollowingBoundary:
                    FollowBoundary(path);
                    break;
            }

            if (path.Count > 10000) // Safety break to prevent infinite loops in complex scenarios
            {
                Console.WriteLine("Safety break: Potential infinite loop.");
                return path;
            }
        }

        path.Add(_goalPosition);
        return path;
    }

    private void MoveTowardsGoal(List<Point> path)
    {
        Point direction = new Point(_goalPosition.X - _robotPosition.X, _goalPosition.Y - _robotPosition.Y);
        double magnitude = Math.Sqrt(direction.X * direction.X + direction.Y * direction.Y);
        Point normalizedDirection = new Point(direction.X / magnitude, direction.Y / magnitude);
        Point nextPosition = new Point(_robotPosition.X + normalizedDirection.X * _stepSize, _robotPosition.Y + normalizedDirection.Y * _stepSize);

        if (IsObstacleBlocking(_robotPosition, nextPosition, _obstacles))
        {
            _currentState = State.FollowingBoundary;
            _hitPoint = _robotPosition;
            _currentBoundary = FindBlockingObstacle(_robotPosition, nextPosition, _obstacles);
            if (_currentBoundary.Any())
            {
                _dMin = double.MaxValue; // Reset dMin
                _dLeave = Point.Distance(_robotPosition, _goalPosition);
                DetermineBoundaryFollowingDirection();
            }
            else
            {
                Console.WriteLine("Error: Could not identify blocking obstacle.");
                return;
            }
        }
        else
        {
            _robotPosition = nextPosition;
            path.Add(_robotPosition);
        }
    }

    private void FollowBoundary(List<Point> path)
    {
        if (!_currentBoundary.Any())
        {
            Console.WriteLine("Error: No boundary to follow.");
            _currentState = State.MovingToGoal; // Fallback
            return;
        }

        // Find the closest point on the boundary to the current robot position
        Point closestBoundaryPoint = FindClosestPointOnPolygon(_robotPosition, _currentBoundary);
        int currentIndex = _currentBoundary.IndexOf(closestBoundaryPoint);
        if (currentIndex == -1)
        {
            // Handle the case where the closest point isn't directly a vertex
            // You might need a more sophisticated method to find your position on the edge
            Console.WriteLine("Warning: Robot not exactly on boundary vertex.");
            return;
        }

        // Determine the next point to move to along the boundary
        int nextIndex = _followClockwise ? (currentIndex + 1) % _currentBoundary.Count : (currentIndex - 1 + _currentBoundary.Count) % _currentBoundary.Count;
        Point nextBoundaryPoint = _currentBoundary[nextIndex];

        // Move a small step towards the next boundary point
        Point moveDirection = new Point(nextBoundaryPoint.X - _robotPosition.X, nextBoundaryPoint.Y - _robotPosition.Y);
        double magnitude = Math.Sqrt(moveDirection.X * moveDirection.X + moveDirection.Y * moveDirection.Y);
        Point normalizedDirection = new Point(moveDirection.X / magnitude, moveDirection.Y / magnitude);
        Point nextPosition = new Point(_robotPosition.X + normalizedDirection.X * _stepSize, _robotPosition.Y + normalizedDirection.Y * _stepSize);

        // Stay on the boundary (simplified - might need more robust boundary following)
        _robotPosition = nextPosition;
        path.Add(_robotPosition);

        double currentDistanceToGoal = Point.Distance(_robotPosition, _goalPosition);
        _dMin = Math.Min(_dMin, currentDistanceToGoal);
        _dLeave = currentDistanceToGoal;

        // Check conditions to leave the boundary
        if (!IsObstacleBlocking(_robotPosition, _goalPosition, _obstacles) || _dLeave < _dMin)
        {
            _currentState = State.MovingToGoal;
        }
    }

    private List<Point> FindBlockingObstacle(Point start, Point end, List<List<Point>> obstacles)
    {
        foreach (var obstacle in obstacles)
        {
            for (int i = 0; i < obstacle.Count; i++)
            {
                Point p1 = obstacle[i];
                Point p2 = obstacle[(i + 1) % obstacle.Count];
                if (SegmentsIntersect(start, end, p1, p2))
                {
                    return obstacle;
                }
            }
        }
        return new List<Point>();
    }

    private bool IsObstacleBlocking(Point start, Point end, List<List<Point>> obstacles)
    {
        foreach (var obstacle in obstacles)
        {
            for (int i = 0; i < obstacle.Count; i++)
            {
                Point p1 = obstacle[i];
                Point p2 = obstacle[(i + 1) % obstacle.Count];
                if (SegmentsIntersect(start, end, p1, p2))
                {
                    return true;
                }
            }
        }
        return false;
    }

    // Simple line segment intersection check
    private bool SegmentsIntersect(Point a, Point b, Point c, Point d)
    {
        double Denominator = (b.X - a.X) * (d.Y - c.Y) - (b.Y - a.Y) * (d.X - c.X);
        if (Denominator == 0)
            return false;

        double t = ((c.X - a.X) * (d.Y - c.Y) - (c.Y - a.Y) * (d.X - c.X)) / Denominator;
        double u = -((a.X - c.X) * (b.Y - a.Y) - (a.Y - c.Y) * (b.X - a.X)) / Denominator;

        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private Point FindClosestPointOnPolygon(Point point, List<Point> polygon)
    {
        if (!polygon.Any())
        {
            return point; // Or throw an exception
        }

        Point closestPoint = polygon[0];
        double minDistance = Point.Distance(point, closestPoint);

        for (int i = 1; i < polygon.Count; i++)
        {
            double distance = Point.Distance(point, polygon[i]);
            if (distance < minDistance)
            {
                minDistance = distance;
                closestPoint = polygon[i];
            }
        }
        return closestPoint;
    }

    private void DetermineBoundaryFollowingDirection()
    {
        // Check both clockwise and counter-clockwise directions briefly
        // to see which one leads to potentially closer points to the goal

        // Simulate moving a small step clockwise
        int currentIndex = _currentBoundary.FindIndex(p => Math.Abs(p.X - _hitPoint.X) < 1e-6 && Math.Abs(p.Y - _hitPoint.Y) < 1e-6);
        if (currentIndex == -1) return; // Should not happen

        int nextClockwiseIndex = (currentIndex + 1) % _currentBoundary.Count;
        Point nextClockwisePoint = _currentBoundary[nextClockwiseIndex];
        double distClockwise = Point.Distance(nextClockwisePoint, _goalPosition);

        int nextCounterClockwiseIndex = (currentIndex - 1 + _currentBoundary.Count) % _currentBoundary.Count;
        Point nextCounterClockwisePoint = _currentBoundary[nextCounterClockwiseIndex];
        double distCounterClockwise = Point.Distance(nextCounterClockwisePoint, _goalPosition);

        _followClockwise = distClockwise < distCounterClockwise;
    }
}

class Program
{
    static void Main(string[] args)
    {
        // Define start and goal positions
        Point start = new Point(1, 1);
        Point goal = new Point(8, 8);

        // Define obstacles as lists of points (representing polygon vertices)
        var obstacles = new List<List<Point>>
            {
                new List<Point> { new Point(3, 3), new Point(5, 3), new Point(5, 5), new Point(3, 5) },
                new List<Point> { new Point(6, 6), new Point(8, 6), new Point(8, 7), new Point(6, 7) }
            };

        // Create the Tangent Bug algorithm instance
        var tangentBug = new TangentBugAlgorithm(start, goal, obstacles, sensorRange: 5, stepSize: 0.2);

        // Find the path
        var path = tangentBug.FindPath();

        // Print the path
        Console.WriteLine("Path found:");
        foreach (var point in path)
        {
            Console.WriteLine($"({point.X:F2}, {point.Y:F2})");
        }
    }
}