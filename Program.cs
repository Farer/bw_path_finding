using System.Collections.Concurrent;
using System.Diagnostics;
using System.Runtime.Serialization;

namespace bw_path_finding;
public enum PathCase
{
    reachableDirectly,
    reachableAlongOneDirection,
    notReachable,
    noPath,
}
public enum Location
{
    unknown,
    same,
    up,
    upRight,
    right,
    downRight,
    down,
    downLeft,
    left,
    upLeft
}
public enum Direction
{
    none,
    up,
    down,
    left,
    right,
}
public class PathFinder(
        (int X, int Y) tileRangeStart,
        (int X, int Y) tileRangeEnd,
        (int X, int Y) start,
        (int X, int Y) goal,
        int sightRange,
        int moveRange,
        (int X, int Y) previousHitObstacle,
        HashSet<(int X, int Y)> closed,
        HashSet<(int X, int Y)> obstacles,
        bool drawMap = true
)
{
    public (int X, int Y) TileRangeStart = tileRangeStart;
    public (int X, int Y) TileRangeEnd = tileRangeEnd;
    public (int X, int Y) Start = start;
    public (int X, int Y) Goal = goal;
    public int SightRange = sightRange;
    public int MoveRange = moveRange;
    public (int X, int Y) PreviousHitObstacle = previousHitObstacle;
    public HashSet<(int X, int Y)> Obstacles = obstacles;
    public HashSet<(int X, int Y)> Closed = closed;
    public int MapWidth = Math.Abs(tileRangeEnd.X - tileRangeStart.X);
    public int MapHeight = Math.Abs(tileRangeEnd.Y - tileRangeStart.Y);
    public bool DrawMap = drawMap;
    public (int X, int Y) HitObstacle = (-1, -1);
    public List<(int X, int Y)> Waypoints = [];
    // from, to, (isReachableDirectly, hitObstacle, path)
    private readonly ConcurrentDictionary<((int X, int Y) from, (int X, int Y) to), (bool, (int X, int Y), List<(int X, int Y)>)> CacheReachableDirectly = new();
    private (int X, int Y) PreviousOrigin = (-1, -1);
    public long ElapsedTime = 0L;
    public List<(int X, int Y)> GetAllValidEdgesFromHitObstacle((int X, int Y) origin, (int X, int Y) hitObstacle, int sightRange, HashSet<(int X, int Y)> obstacles)
    {
        var validEdges = new List<(int X, int Y)>();
        var targetEdgeQueue = new Queue<((int X, int Y) point, int distance)>();
        var visited = new HashSet<(int X, int Y)>();

        targetEdgeQueue.Enqueue((hitObstacle, 0));
        visited.Add(hitObstacle);

        while (targetEdgeQueue.Count > 0)
        {
            // Dequeue the current point and distance
            var (currentTargetEdgePoint, distance) = targetEdgeQueue.Dequeue();

            // Skip if the current point is out of bounds or too far from the start
            if (distance > sightRange) { continue; }

            // Check if the current point is a valid edge
            var nearTiles = GetNeighborTiles(currentTargetEdgePoint);
            foreach (var tile in nearTiles)
            {
                if (!obstacles.Contains(tile))
                {
                    if (IsReachableDirectly(origin, currentTargetEdgePoint).Item1)
                    {
                        validEdges.Add(currentTargetEdgePoint);
                        break;
                    }
                }
            }

            // Explore adjacent tiles
            var adjacentTilesOfTargetTile = GetAdjacentTiles(currentTargetEdgePoint);
            foreach (var targetEdgeNeighbor in adjacentTilesOfTargetTile)
            {
                if (!IsOutOfBound(targetEdgeNeighbor) && obstacles.Contains(targetEdgeNeighbor) && !visited.Contains(targetEdgeNeighbor))
                {
                    visited.Add(targetEdgeNeighbor);
                    targetEdgeQueue.Enqueue((targetEdgeNeighbor, distance + 1));
                }
            }
        }

        return validEdges;
    }
    public bool HasNeighborOutOfBound((int X, int Y) targetTile)
    {
        var nearTiles = GetNeighborTiles(targetTile);
        return nearTiles.Any(IsOutOfBound);
    }
    public bool IsOutOfBound((int X, int Y) targetTile)
    {
        return targetTile.X < TileRangeStart.X || targetTile.X >= TileRangeEnd.X || targetTile.Y < TileRangeStart.Y || targetTile.Y >= TileRangeEnd.Y;
    }
    public (bool, HashSet<(int X, int Y)>) CheckReachableAlongOneDirection_1((int X, int Y) origin, (int X, int Y) target)
    {
        // Console.WriteLine("CheckReachableAlongOneDirection_1 - " + "origin: " + origin + ", target: " + target);
        var wayPoint = new HashSet<(int X, int Y)>();
        bool isReachable = false;

        if (IsNeighborTile(origin, target)) { return (true, [origin, target]); }

        int plusMinusX = target.X < origin.X ? -1 : 1;
        int plusMinusY = target.Y < origin.Y ? -1 : 1;

        if (Math.Abs(origin.X - target.X) == 1 && Math.Abs(origin.Y - target.Y) == 1)
        {
            if (!Obstacles.Contains((target.X, origin.Y)))
            {
                wayPoint = [(target.X, origin.Y)];
                return (true, wayPoint);
            }
            else if (!Obstacles.Contains((origin.X, target.Y)))
            {
                wayPoint = [(origin.X, target.Y)];
                return (true, wayPoint);
            }
            else { return (false, wayPoint); }
        }

        if (Math.Abs(origin.X - target.X) == 1)
        {
            isReachable = true;
            if (Obstacles.Contains((target.X, origin.Y)))
            {
                isReachable = false;
            }
            else
            {
                for (var toY = origin.Y; toY != target.Y; toY += plusMinusY)
                {
                    if (Obstacles.Contains((target.X, toY)))
                    {
                        isReachable = false;
                        break;
                    }
                }
            }

            if (isReachable)
            {
                wayPoint = [(target.X, origin.Y)];
                return (isReachable, wayPoint);
            }

            if (!isReachable)
            {
                for (var toY = origin.Y + plusMinusY; toY != target.Y; toY += plusMinusY)
                {
                    if (Obstacles.Contains((origin.X, toY)))
                    {
                        isReachable = false;
                        break;
                    }
                    else
                    {
                        (int X, int Y) tempWayPoint = (origin.X, toY);
                        var tmpResult_1 = CheckReachableAlongOneDirection_1((origin.X, toY), (target.X, target.Y));
                        if (tmpResult_1.Item1)
                        {
                            isReachable = true;
                            var fixedY = tmpResult_1.Item2.Last().Y;
                            if (tempWayPoint.Y != fixedY) { wayPoint = [.. tmpResult_1.Item2]; }
                            else { wayPoint = [tempWayPoint, .. tmpResult_1.Item2,]; }
                            // foreach (var item in wayPoint) { Console.WriteLine($"wayPoint: {item}"); }
                            return (isReachable, wayPoint);
                        }
                    }
                }
            }
        }
        else if (Math.Abs(origin.Y - target.Y) == 1)
        {
            isReachable = true;
            if (Obstacles.Contains((origin.X, target.Y)))
            {
                isReachable = false;
            }
            else
            {
                for (var toX = origin.X; toX != target.X; toX += plusMinusX)
                {
                    if (Obstacles.Contains((toX, target.Y)))
                    {
                        isReachable = false;
                        break;
                    }
                }
            }

            if (isReachable)
            {
                wayPoint = [(origin.X, target.Y)];
                return (isReachable, wayPoint);
            }


            if (!isReachable)
            {
                for (var toX = origin.X + plusMinusX; toX != target.X; toX += plusMinusX)
                {
                    if (Obstacles.Contains((toX, origin.Y)))
                    {
                        isReachable = false;
                        break;
                    }
                    else
                    {
                        (int X, int Y) tempWayPoint = (toX, origin.Y);
                        var tmpResult_1 = CheckReachableAlongOneDirection_1((toX, origin.Y), (target.X, target.Y));
                        if (tmpResult_1.Item1)
                        {
                            isReachable = true;
                            var fixedX = tmpResult_1.Item2.Last().X;
                            if (tempWayPoint.X != fixedX) { wayPoint = [.. tmpResult_1.Item2]; }
                            else { wayPoint = [tempWayPoint, .. tmpResult_1.Item2,]; }
                            // foreach (var item in wayPoint) { Console.WriteLine($"wayPoint: {item}"); }
                            return (isReachable, wayPoint);
                        }
                    }
                }
            }
        }

        return (isReachable, wayPoint);
    }
    public (bool, HashSet<(int X, int Y)>) CheckReachableAlongOneDirectionByDistance((int X, int Y) origin, (int X, int Y) target, int distance)
    {
        // Console.WriteLine($"CheckReachableAlongOneDirectionByDistance - distance: {distance}, origin: {origin}, target: {target}");

        if (distance <= 0) { return (false, []); }

        if (distance == 1) { return CheckReachableAlongOneDirection_1(origin, target); }

        var wayPoint = new HashSet<(int X, int Y)>();
        int plusMinusX = target.X < origin.X ? -1 : 1;
        int plusMinusY = target.Y < origin.Y ? -1 : 1;

        if (Math.Abs(origin.X - target.X) == distance)
        {
            var towardTargetX = origin.X + plusMinusX;
            if (!Obstacles.Contains((towardTargetX, origin.Y)))
            {
                var isReachableResult = CheckReachableAlongOneDirectionByDistance((towardTargetX, origin.Y), target, distance - 1);
                if (isReachableResult.Item1)
                {
                    return (true, [(towardTargetX, origin.Y), .. isReachableResult.Item2]);
                }
            }

            for (var towardTargetY = origin.Y + plusMinusY; towardTargetY != target.Y + plusMinusY; towardTargetY += plusMinusY)
            {
                if (!Obstacles.Contains((origin.X, towardTargetY)))
                {
                    (int X, int Y) tempWayPoint = (origin.X, towardTargetY);
                    if (!Obstacles.Contains((towardTargetX, towardTargetY)))
                    {
                        var isReachableResult = CheckReachableAlongOneDirectionByDistance((towardTargetX, towardTargetY), target, distance - 1);
                        if (isReachableResult.Item1)
                        {
                            return (true, [tempWayPoint, (towardTargetX, towardTargetY), .. isReachableResult.Item2]);
                        }
                    }
                }
                else { break; }
            }
        }

        if (Math.Abs(origin.Y - target.Y) == distance)
        {
            var towardTargetY = origin.Y + plusMinusY;
            if (!Obstacles.Contains((origin.X, towardTargetY)))
            {
                var isReachableResult = CheckReachableAlongOneDirectionByDistance((origin.X, towardTargetY), target, distance - 1);
                if (isReachableResult.Item1)
                {
                    return (true, [(origin.X, towardTargetY), .. isReachableResult.Item2]);
                }
            }

            for (var towardTargetX = origin.X + plusMinusX; towardTargetX != target.X + plusMinusX; towardTargetX += plusMinusX)
            {
                if (!Obstacles.Contains((towardTargetX, origin.Y)))
                {
                    (int X, int Y) tempWayPoint = (towardTargetX, origin.Y);
                    if (!Obstacles.Contains((towardTargetX, towardTargetY)))
                    {
                        var isReachableResult = CheckReachableAlongOneDirectionByDistance((towardTargetX, towardTargetY), target, distance - 1);
                        if (isReachableResult.Item1)
                        {
                            return (true, [tempWayPoint, (towardTargetX, towardTargetY), .. isReachableResult.Item2]);
                        }
                    }
                }
                else { break; }
            }
        }

        return (false, []);
    }
    /// <summary>
    /// Checks if the end point is reachable from the start point without crossing any obstacles using a straight-line path (Bresenham's line algorithm).
    /// </summary>
    public (bool, (int X, int Y), List<(int X, int Y)>) IsReachableDirectly((int X, int Y) from, (int X, int Y) to)
    {
        return CacheReachableDirectly.GetOrAdd((from, to), _ =>
        {
            int x = from.X;
            int y = from.Y;
            int dx = Math.Abs(to.X - from.X);
            int dy = Math.Abs(to.Y - from.Y);
            int sx = from.X < to.X ? 1 : -1;
            int sy = from.Y < to.Y ? 1 : -1;
            int err = dx - dy;
            bool isReachable = true;
            var path = new List<(int X, int Y)>();
            (int X, int Y) firstObstacle = (-1, -1);

            while (true)
            {
                if (Obstacles.Contains((x, y)) && (x, y) != to)
                {
                    isReachable = false;
                    firstObstacle = (x, y);
                    break;
                }

                if (x == to.X && y == to.Y) { break; }

                path.Add((x, y));
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
            return (isReachable, firstObstacle, path);
        });
    }
    /// <summary>
    /// Returns the adjacent tiles of a given tile.
    /// </summary>
    /// <param name="tile">The reference tile coordinates.</param>
    /// <returns>A list of adjacent tile coordinates.</returns>
    public static List<(int X, int Y)> GetAdjacentTiles((int X, int Y) tile)
    {
        var tmp = new List<(int X, int Y)> {
            (tile.X - 1, tile.Y),
            (tile.X + 1, tile.Y),
            (tile.X, tile.Y - 1),
            (tile.X, tile.Y + 1),
            (tile.X - 1, tile.Y - 1),
            (tile.X - 1, tile.Y + 1),
            (tile.X + 1, tile.Y - 1),
            (tile.X + 1, tile.Y + 1)
        };
        return tmp;
    }
    public static List<(int X, int Y)> GetNeighborTiles((int X, int Y) tile)
    {
        return [
            (tile.X - 1, tile.Y),
            (tile.X + 1, tile.Y),
            (tile.X, tile.Y - 1),
            (tile.X, tile.Y + 1)
        ];
    }
    public static Direction GetDirectionFromTarget((int X, int Y) origin, (int X, int Y) target)
    {
        var diffX = target.X - origin.X;
        var diffY = target.Y - origin.Y;
        if (diffX == 0 && diffY == 1) { return Direction.up; }
        else if (diffX == 0 && diffY == -1) { return Direction.down; }
        else if (diffX == 1 && diffY == 0) { return Direction.left; }
        else if (diffX == -1 && diffY == 0) { return Direction.right; }
        else { return Direction.none; }
    }
    public static bool IsNeighborTile((int X, int Y) tile, (int X, int Y) target)
    {
        if (Math.Abs(tile.X - target.X) == 0 && Math.Abs(tile.Y - target.Y) == 1) { return true; }
        if (Math.Abs(tile.X - target.X) == 1 && Math.Abs(tile.Y - target.Y) == 0) { return true; }
        else { return false; }
    }
    public bool IfHasValidDetourPoint((int X, int Y) origin, (int X, int Y) target) {
        var adjacentTiles = GetAdjacentTiles(target);
        foreach(var tile in adjacentTiles) {
            if(IsReachableDirectly(origin, tile).Item1) {
                return true;
            }
        }
        return false;
    }
    public static ((int X, int Y), (int X, int Y))? FindOuterMostTiles((int X, int Y) originTile, List<(int X, int Y)> validEdgeTiles) 
    {
        // Validate input: if no valid tile is provided, return null.
        // If there's only one tile, return that tile for both leftmost and rightmost.
        if (validEdgeTiles == null || validEdgeTiles.Count == 0) { return null; }
        if (validEdgeTiles.Count == 1) { return (validEdgeTiles[0], validEdgeTiles[0]); }
        
        // Calculate the angle from the origin to each valid edge tile
        // and store the results in a list.
        var angles = validEdgeTiles.Select(edge => new {
            Edge = edge,
            Angle = Math.Atan2(edge.Y - originTile.Y, edge.X - originTile.X)
        }).ToList();
        
        // Sort the list by angle in ascending order.
        angles.Sort((a, b) => a.Angle.CompareTo(b.Angle));
        
        const double epsilon = 1e-10; // Tolerance for floating-point comparison

        // Select the leftmost candidate (smallest angle).
        // Iterate from the beginning until the angle deviates more than epsilon.
        double leftmostAngle = angles[0].Angle;
        var leftCandidate = angles[0].Edge;
        double leftMaxDistance = CalculateDistance(originTile, leftCandidate);
        
        for (int i = 1; i < angles.Count; i++)
        {
            if (Math.Abs(angles[i].Angle - leftmostAngle) <= epsilon)
            {
                double distance = CalculateDistance(originTile, angles[i].Edge);
                if (distance > leftMaxDistance)
                {
                    leftMaxDistance = distance;
                    leftCandidate = angles[i].Edge;
                }
            }
            else
            {
                break; // Exit the loop when encountering a significantly different angle.
            }
        }
        
        // Select the rightmost candidate (largest angle).
        // Iterate from the end until the angle deviates more than epsilon.
        double rightmostAngle = angles[^1].Angle;
        var rightCandidate = angles[^1].Edge;
        double rightMaxDistance = CalculateDistance(originTile, rightCandidate);
        
        for (int i = angles.Count - 2; i >= 0; i--) {
            if (Math.Abs(angles[i].Angle - rightmostAngle) <= epsilon) {
                double distance = CalculateDistance(originTile, angles[i].Edge);
                if (distance > rightMaxDistance) {
                    rightMaxDistance = distance;
                    rightCandidate = angles[i].Edge;
                }
            }
            else {
                break; // Exit the loop when the angle deviates beyond the tolerance.
            }
        }
        
        return (leftCandidate, rightCandidate);
    }
    public ((int x, int y) leftMostTarget, (int x, int y) rightMostTarget) FindExtremeAngleTargets(
        (int X, int Y) origin,
        IEnumerable<(int X, int Y)> targets,
        bool isReachable = false
    )
    {
        var finalCandidates = new List<(int X, int Y)>();
        foreach (var item in targets)
        {
            if(Obstacles.Contains(item)) { continue; }
            if (isReachable) {
                if (origin != item && IsReachableDirectly(origin, item).Item1) { finalCandidates.Add(item); }
            }
            else { finalCandidates.Add(item); }
        }
        // calculate the angle (in radians)
        List<(double angle, (int x, int y) target)> angles = [.. finalCandidates.Select(target => {
            double angle = Math.Atan2(target.Y - origin.Y, target.X - origin.X);
            return (angle, target);
        })];

        // calculate the leftmost and rightmost targets
        var leftMost = angles.OrderBy(a => a.angle).First();
        var rightMost = angles.OrderByDescending(a => a.angle).First();

        return (leftMost.target, rightMost.target);
    }

    // Helper function to calculate the Euclidean distance between two points.
    public static double CalculateDistance((int X, int Y) p1, (int X, int Y) p2)
    {
        int dx = p1.X - p2.X;
        int dy = p1.Y - p2.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }
    private Direction DetermineHitObstacleContinueToOneDirection()
    {
        var diffX = HitObstacle.X - PreviousHitObstacle.X;
        var diffY = HitObstacle.Y - PreviousHitObstacle.Y;
        if (diffX == 1 && diffY == 0) { return Direction.right; }
        else if (diffX == -1 && diffY == 0) { return Direction.left; }
        else if (diffX == 0 && diffY == 1) { return Direction.down; }
        else if (diffX == 0 && diffY == -1) { return Direction.up; }
        return Direction.none;
    }
    public static (int X, int Y) FindClosestDetourToTarget((int X, int Y) origin, (int X, int Y) target, List<(int X, int Y)> detours)
    {
        // calculate the angle of the goal
        double targetAngle = Math.Atan2(target.Y - origin.Y, target.X - origin.X);

        (int X, int Y) closestDetour = (-1, -1);
        double smallestAngleDifference = double.MaxValue;

        foreach (var detour in detours)
        {
            // calculate the angle of the detour
            double detourAngle = Math.Atan2(detour.Y - origin.Y, detour.X - origin.X);

            // calculate the difference between the goal angle and the detour angle (absolute value)
            double angleDifference = Math.Abs(targetAngle - detourAngle);

            // adjust the angle difference so that it does not exceed 180 degrees
            if (angleDifference > Math.PI)
            {
                angleDifference = 2 * Math.PI - angleDifference;
            }

            // pick the detour with the smallest angle difference
            if (angleDifference < smallestAngleDifference)
            {
                smallestAngleDifference = angleDifference;
                closestDetour = detour;
            }
        }
        return closestDetour;
    }
    public void DisplayMap(List<(int X, int Y)>? path, List<(int X, int Y)>? detourPoint, (int X, int Y)? hitObstacle)
    {
        string[,] map = new string[MapWidth + 1, MapHeight];
        string charLand = "·".PadLeft(2);
        string charObstacle = "🟢";
        string charStart = "🐰".PadRight(2);
        string charGoal = "🌱";
        string charPath = "🔸";
        string charHit = "💥";
        string charDetour = "💠";

        // Initialize map
        for (int y = 0; y < MapHeight; y++)
        {
            for (int x = 0; x <= MapWidth; x++)
            {
                if (x == MapWidth) { map[x, y] = y.ToString(); }
                else { map[x, y] = charLand; }
            }
        }

        map[Start.X, Start.Y] = charStart;
        map[Goal.X, Goal.Y] = charGoal;

        foreach (var (X, Y) in Obstacles)
        {
            if (Y >= 0 && Y < MapHeight && X >= 0 && X < MapWidth)
            {
                map[X, Y] = charObstacle;
            }
        }

        if (path != null)
        {
            foreach (var (X, Y) in path)
            {
                if (Y >= 0 && Y < MapHeight && X >= 0 && X < MapWidth && map[X, Y] == charLand)
                {
                    map[X, Y] = charPath;
                }
            }
        }

        if (detourPoint != null && detourPoint.Count > 0)
        {
            foreach (var (X, Y) in detourPoint)
            {
                if (Y >= 0 && Y < MapHeight && X >= 0 && X < MapWidth) { map[X, Y] = charDetour; }
            }
        }

        if (hitObstacle.HasValue)
        {
            if (hitObstacle.Value.Y >= 0 && hitObstacle.Value.Y < MapHeight && hitObstacle.Value.X >= 0 && hitObstacle.Value.X < MapWidth)
            {
                map[hitObstacle.Value.X, hitObstacle.Value.Y] = charHit;
            }
        }

        Console.WriteLine();
        for (int x = 0; x < MapWidth; x++)
        {
            var pad = 2;
            Console.Write(x.ToString().PadLeft(pad) + " ");
        }
        Console.WriteLine();
        for (int y = 0; y < MapHeight; y++)
        {
            for (int x = 0; x <= MapWidth; x++)
            {
                Console.Write(map[x, y] + " ");
            }
            Console.WriteLine();
        }
    }
    private static Location DetermineTargetLocationFromOrigin((int X, int Y) origin, (int X, int Y) target)
    {
        if (origin == target) { return Location.same; }
        else if (origin.X == target.X && origin.Y < target.Y) { return Location.up; }
        else if (origin.X == target.X && origin.Y > target.Y) { return Location.down; }
        else if (origin.Y == target.Y && origin.X < target.X) { return Location.right; }
        else if (origin.Y == target.Y && origin.X > target.X) { return Location.left; }
        else if (origin.X < target.X && origin.Y > target.Y) { return Location.upRight; }
        else if (origin.X < target.X && origin.Y < target.Y) { return Location.downRight; }
        else if (origin.X > target.X && origin.Y > target.Y) { return Location.upLeft; }
        else if (origin.X > target.X && origin.Y < target.Y) { return Location.downLeft; }
        else { return Location.unknown; }
    }
    private List<(int X, int Y)> GetObstacleCheckRangeByTargetLocation((int X, int Y) origin, Location targetLocation)
    {
        if (targetLocation == Location.unknown || targetLocation == Location.same)
        {
            return [(-1, -1)];
        }
        var adjacentTiles = GetAdjacentTiles(origin);

        var adjustedTiles = new List<(int X, int Y)>();
        var plusMinusX = 0; var plusMinusY = 0;
        switch (targetLocation)
        {
            case Location.up: plusMinusX = 0; plusMinusY = -2; break;
            case Location.down: plusMinusX = 0; plusMinusY = 2; break;
            case Location.left: plusMinusX = -2; plusMinusY = 0; break;
            case Location.right: plusMinusX = 2; plusMinusY = 0; break;
            case Location.upRight: plusMinusX = 2; plusMinusY = -2; break;
            case Location.downRight: plusMinusX = 2; plusMinusY = 2; break;
            case Location.upLeft: plusMinusX = -2; plusMinusY = -2; break;
            case Location.downLeft: plusMinusX = -2; plusMinusY = 2; break;
        }
        adjacentTiles.Add((origin.X, origin.Y));

        foreach (var (X, Y) in adjacentTiles)
        {
            var newPos = (X + plusMinusX, Y + plusMinusY);
            if (!IsOutOfBound(newPos)) { adjustedTiles.Add(newPos); }
        }
        return adjustedTiles;
    }
    private bool CheckIfObstaclesAreNear((int X, int Y) origin, (int X, int Y) target)
    {
        var targetLocation = DetermineTargetLocationFromOrigin(origin, target);
        var obstacleCheckRange = GetObstacleCheckRangeByTargetLocation(origin, targetLocation);
        var count = 0;
        foreach (var pos in obstacleCheckRange)
        {
            if (Obstacles.Contains(pos)) { count++; }
        }
        // return count >= 3;
        return count >= 1;
    }
    private PathCase ProceedBasicSearch()
    {
        // Console.WriteLine();
        // Console.WriteLine("start: " + Start + " -> Goal:" + Goal);
        // Console.WriteLine();

        // Stopwatch stopwatch = new();
        // stopwatch.Start();
        var checkReachableDirectlyResult = IsReachableDirectly(Start, Goal);
        // stopwatch.Stop(); ElapsedTime += stopwatch.ElapsedMilliseconds;
        if (checkReachableDirectlyResult.Item1)
        {
            // Console.WriteLine("Goal is reachable directly");
            // Console.WriteLine();
            // Console.WriteLine($"Elapsed time: {stopwatch.ElapsedMilliseconds}ms");
            // DisplayMap(checkReachableDirectlyResult.Item3, null, null);

            Waypoints = [checkReachableDirectlyResult.Item3.Last()];
            return PathCase.reachableDirectly;
        }
        else
        {
            HitObstacle = checkReachableDirectlyResult.Item2;
        }

        // if obstacles are near
        if (CheckIfObstaclesAreNear(Start, Goal))
        {
            var distanceX = Math.Abs(Start.X - Goal.X);
            var distanceY = Math.Abs(Start.Y - Goal.Y);
            var leastDistance = Math.Min(distanceX, distanceY);

            if (leastDistance <= 10)
            {
                // Check reachability along one direction with increasing distances
                var checkReachableAlongOneDirectionResult = CheckReachableAlongOneDirectionByDistance(Start, Goal, leastDistance);
                if (checkReachableAlongOneDirectionResult.Item1)
                {
                    // stopwatch.Stop(); ElapsedTime += stopwatch.ElapsedMilliseconds;
                    // Console.WriteLine($"Goal is reachable along one direction with distance: {leastDistance}");
                    // Console.WriteLine();
                    // Console.WriteLine($"Elapsed time: {stopwatch.ElapsedMilliseconds}ms");
                    Waypoints = [.. checkReachableAlongOneDirectionResult.Item2];
                    return PathCase.reachableAlongOneDirection;
                }
            }

            // stopwatch.Stop(); ElapsedTime += stopwatch.ElapsedMilliseconds;
            // Console.WriteLine();
            // Console.WriteLine($"Elapsed time: {stopwatch.ElapsedMilliseconds}ms");

            HitObstacle = checkReachableDirectlyResult.Item2;
            // Console.WriteLine();
            // Console.WriteLine($"Goal is not reachable. Hit obstacle at {checkReachableDirectlyResult.Item2}");
            // DisplayMap(checkReachableDirectlyResult.Item3, null, checkReachableDirectlyResult.Item2);
        }


        return PathCase.notReachable;
    }
    private (int X, int Y) GetEdgeToHitObstacleDirection((int X, int Y) origin, Direction hitObstacleDirection)
    {
        var directionOfOrigin = GetDirectionFromTarget(origin, HitObstacle);
        (int X, int Y) edgeToDirection = (-1, -1);

        (int X, int Y) finalPos = (-1, -1);
        if (hitObstacleDirection == Direction.up)
        {
            for (var y = HitObstacle.Y; y >= 0; y--)
            {
                finalPos = (HitObstacle.X, y);
                if (Obstacles.Contains(finalPos))
                {
                    (int X, int Y) originSide = (-1, -1);
                    if (directionOfOrigin == Direction.left) { originSide = (HitObstacle.X - 1, y); }
                    else if (directionOfOrigin == Direction.right) { originSide = (HitObstacle.X + 1, y); }
                    if (!Obstacles.Contains(originSide))
                    {
                        edgeToDirection = (HitObstacle.X, y);
                        continue;
                    }
                }
                break;
            }
        }
        else if (hitObstacleDirection == Direction.down)
        {
            for (var y = HitObstacle.Y; y < MapHeight; y++)
            {
                finalPos = (HitObstacle.X, y);
                if (Obstacles.Contains(finalPos))
                {
                    (int X, int Y) originSide = (-1, -1);
                    if (directionOfOrigin == Direction.left) { originSide = (HitObstacle.X - 1, y); }
                    else if (directionOfOrigin == Direction.right) { originSide = (HitObstacle.X + 1, y); }
                    if (!Obstacles.Contains(originSide))
                    {
                        edgeToDirection = (HitObstacle.X, y);
                        continue;
                    }
                }
                break;
            }
        }
        else if (hitObstacleDirection == Direction.left)
        {
            for (var x = HitObstacle.X; x >= 0; x--)
            {
                finalPos = (x, HitObstacle.Y);
                if (Obstacles.Contains(finalPos))
                {
                    (int X, int Y) originSide = (-1, -1);
                    if (directionOfOrigin == Direction.up) { originSide = (x, HitObstacle.Y - 1); }
                    else if (directionOfOrigin == Direction.down) { originSide = (x, HitObstacle.Y + 1); }
                    if (!Obstacles.Contains(originSide))
                    {
                        edgeToDirection = (x, HitObstacle.Y);
                        continue;
                    }
                }
                break;
            }
        }
        else if (hitObstacleDirection == Direction.right)
        {
            for (var x = HitObstacle.X; x < MapWidth; x++)
            {
                finalPos = (x, HitObstacle.Y);
                if (Obstacles.Contains(finalPos))
                {
                    (int X, int Y) originSide = (-1, -1);
                    if (directionOfOrigin == Direction.up) { originSide = (x, HitObstacle.Y - 1); }
                    else if (directionOfOrigin == Direction.down) { originSide = (x, HitObstacle.Y + 1); }
                    if (!Obstacles.Contains(originSide))
                    {
                        edgeToDirection = (x, HitObstacle.Y);
                        continue;
                    }
                }
                break;
            }
        }
        return edgeToDirection;
    }
    public (int X, int Y) ProceedNormalSearch()
    {
        var distanceFromStartToHitObstacle = CalculateDistance(Start, HitObstacle);
        // Will Move long way to hit obstacle
        if(distanceFromStartToHitObstacle > 10) { PreviousOrigin = Start; }
        // Stopwatch stopwatch = new();
        var hitObstacleDirection = DetermineHitObstacleContinueToOneDirection();

        var allValidEdges = new List<(int X, int Y)>();
        if (hitObstacleDirection != Direction.none)
        {
            var edgeToHitObstacleDirection = GetEdgeToHitObstacleDirection(Start, hitObstacleDirection);
            if (edgeToHitObstacleDirection != (-1, -1))
            {
                allValidEdges.Add(edgeToHitObstacleDirection);
            }
        }
        else
        {
            allValidEdges = GetAllValidEdgesFromHitObstacle(Start, HitObstacle, SightRange, Obstacles);

        }

        // stopwatch.Stop(); ElapsedTime += stopwatch.ElapsedMilliseconds;
        // Console.WriteLine($"Elapsed time: {stopwatch.ElapsedMilliseconds}ms");
        // Console.WriteLine();

        // if (allValidEdges.Count > 0) { Console.Write($"Valid Edge: "); }
        // foreach (var edge in allValidEdges)
        // {
        //     Console.Write(" " + edge);
        // }
        // Console.WriteLine();

        (int X, int Y) bestDetourPoint;
        if (allValidEdges.Count == 1) {
            bestDetourPoint = GetBestDetourPointWithSingleValidEdge(Start, Goal, allValidEdges[0]);
        }
        else {
            bool isValidEdges = true;
            while(true) {
                var outerMostEdges = FindOuterMostTiles(Start, allValidEdges);
                if(outerMostEdges is null) {
                    Console.WriteLine("No outermost edges");
                    isValidEdges = false;
                    break;
                }
                bool leftIsValid = HasRechableAdjacentTileFromOrigin(Start, outerMostEdges.Value.Item1);
                bool rightIsValid = HasRechableAdjacentTileFromOrigin(Start, outerMostEdges.Value.Item2);
                if(!leftIsValid) { allValidEdges.Remove(outerMostEdges.Value.Item1); }
                if(!rightIsValid) { allValidEdges.Remove(outerMostEdges.Value.Item2); }
                if(!leftIsValid || !rightIsValid) { continue; }
                else { break; }
            }
            if(isValidEdges) { bestDetourPoint = GetFinalDetourPointWithMiultipleValidEdges(Start, allValidEdges); }
            else { bestDetourPoint = (-1, -1); }
        }
        return bestDetourPoint;
    }
    private bool HasRechableAdjacentTileFromOrigin((int X, int Y) origin, (int X, int Y) target) {
        var adjacentTiles = GetAdjacentTiles(target);
        foreach(var tile in adjacentTiles) {
            if(!Obstacles.Contains(tile) && IsReachableDirectly(origin, tile).Item1) {
                return true;
            }
        }
        return false;
    }
    private (int X, int Y) GetBestDetourPointWithSingleValidEdge((int X, int Y) origin, (int X, int Y) target, (int X, int Y) validEdge)
    {
        // Console.WriteLine("Only one valid edge");

        // Stopwatch stopwatch = new();
        // stopwatch.Start();

        var adjacent = GetAdjacentTiles(validEdge);
        var bestDetourCandidatesList = new List<(int X, int Y)>();
        // if (adjacent.Count > 0) { Console.Write("Adjacent: "); }
        foreach (var item in adjacent)
        {
            // Console.Write(" " + item);
            if (Obstacles.Contains(item)) { continue; }
            if (origin == item) { continue; }
            if (IsOutOfBound(item)) { continue; }
            if (IsReachableDirectly(origin, item).Item1) {
                bestDetourCandidatesList.Add(item);
            }
        }
        // Console.WriteLine();

        if(bestDetourCandidatesList.Count == 1) { return bestDetourCandidatesList[0];}

        var outerMostTiles = FindOuterMostTiles(origin, bestDetourCandidatesList);
        if (outerMostTiles is null) { return (-1, -1); }

        var left = outerMostTiles!.Value.Item1;
        var right = outerMostTiles!.Value.Item2;

        (int X, int Y) bestDetourPoint = (-1, -1);

        bool leftIsClosed = IfClosed(left);
        bool rightIsClosed = IfClosed(right);
        if (!leftIsClosed && rightIsClosed) { bestDetourPoint = left; }
        else if (leftIsClosed && !rightIsClosed) { bestDetourPoint = right; }

        if (PreviousOrigin != (-1, -1) && bestDetourPoint == (-1, -1)) {
            var distanceFromPreviousOriginToLeft = CalculateDistance(PreviousOrigin, left);
            var distanceFromPreviousOriginToRight = CalculateDistance(PreviousOrigin, right);
            if (distanceFromPreviousOriginToLeft < distanceFromPreviousOriginToRight) { bestDetourPoint = right; }
            else if (distanceFromPreviousOriginToRight < distanceFromPreviousOriginToLeft) { bestDetourPoint = left; }
        }

        if (bestDetourPoint == (-1, -1))
        {
            var leftValidNeighborCount = GetValidAdjacentCount(left);
            var rightValidNeighborCount = GetValidAdjacentCount(right);
            // Console.WriteLine("leftDetourpointValidNeighborCount: " + leftValidNeighborCount);
            // Console.WriteLine("rightDetourpointValidNeighborCount: " + rightValidNeighborCount);
            if (leftValidNeighborCount > rightValidNeighborCount) { bestDetourPoint = left; }
            else if (rightValidNeighborCount > leftValidNeighborCount) { bestDetourPoint = right; }
        }

        if (bestDetourPoint == (-1, -1))
        {
            var leftDistanceFromStart = CalculateDistance(origin, left);
            var leftDistanceToGoal = CalculateDistance(target, left);
            var leftTotalDistance = leftDistanceFromStart + leftDistanceToGoal;

            var rightDistanceFromStart = CalculateDistance(origin, right);
            var rightDistanceToGoal = CalculateDistance(target, right);
            var rightTotalDistance = rightDistanceFromStart + rightDistanceToGoal;

            // var closeAngleToTargetDetourPoint = FindClosestDetourToTarget(origin, target, [left, right]);

            // Console.WriteLine("[ Left ] from origin: " + leftDistanceFromStart + ", to target: " + leftDistanceToGoal);
            // Console.WriteLine("leftTotalDistance: " + leftTotalDistance);
            // Console.WriteLine("[ Right ] from origin: " + rightDistanceFromStart + ", to target: " + rightDistanceToGoal);
            // Console.WriteLine("rightTotalDistance: " + rightTotalDistance);
            // Console.WriteLine("Closest detour point to target: " + closeAngleToTargetDetourPoint);

            // Console.WriteLine();


            if (leftTotalDistance < rightTotalDistance)
            {
                bestDetourPoint = outerMostTiles!.Value.Item1;
            }
            else if (leftTotalDistance > rightTotalDistance)
            {
                bestDetourPoint = outerMostTiles!.Value.Item2;
            }
            else
            {
                // Console.WriteLine("Same distance");
                var random = new Random();
                var randomIndex = random.Next(0, 2);
                if (randomIndex == 0)
                {
                    bestDetourPoint = outerMostTiles!.Value.Item1;
                }
                else
                {
                    bestDetourPoint = outerMostTiles!.Value.Item2;
                }
            }
        }

        // Console.WriteLine("Final Detour Point: " + finalDetourPoint);
        // stopwatch.Stop(); ElapsedTime += stopwatch.ElapsedMilliseconds;
        // Console.WriteLine();
        // Console.WriteLine($"Elapsed time: {stopwatch.ElapsedMilliseconds}ms");

        return bestDetourPoint;
    }
    private bool IfClosed((int X, int Y) target)
    {
        return Closed.Contains(target);
    }
    private int GetValidAdjacentCount((int X, int Y) target)
    {
        var adjacent = GetAdjacentTiles(target);
        var count = 0;
        foreach (var item in adjacent)
        {
            if (!IsOutOfBound(item) && !Obstacles.Contains(item) && !Closed.Contains(item))
            {
                count++;
            }
        }
        return count;
    }
    private (int X, int Y) GetFinalDetourPointWithMiultipleValidEdges((int X, int Y) origin, List<(int X, int Y)> validEdges)
    {
        // Stopwatch stopwatch = new();
        // stopwatch.Start();

        var outerMostEdges = FindOuterMostTiles(origin, validEdges);
        // Console.WriteLine($"leftmost: {outerMostEdges?.Item1}, rightmost: {outerMostEdges?.Item2}");

        var left = outerMostEdges!.Value.Item1;
        var right = outerMostEdges!.Value.Item2;
        if(left == right) { return left; }

        bool isLeftNearTheBorder = HasNeighborOutOfBound(left);
        bool isRightNearTheBorder = HasNeighborOutOfBound(right);

        var finalTargetEdge = (-1, -1);
        string direction = "";
        if (isLeftNearTheBorder)
        {
            // Console.WriteLine("Left edge is near the border");
            direction = "right";
        }
        else if (isRightNearTheBorder)
        {
            // Console.WriteLine("Right edge is near the border");
            direction = "left";
        }
        else if (!isLeftNearTheBorder && !isRightNearTheBorder)
        {
            if (direction == "")
            {
                var leftValidNeighborCount = GetValidAdjacentCount(left);
                var rightValidNeighborCount = GetValidAdjacentCount(right);
                // Console.WriteLine("leftDetourpointValidNeighborCount: " + leftValidNeighborCount);
                // Console.WriteLine("rightDetourpointValidNeighborCount: " + rightValidNeighborCount);
                if (leftValidNeighborCount > rightValidNeighborCount) { direction = "left"; }
                else if (rightValidNeighborCount > leftValidNeighborCount) { direction = "right"; }
            }

            if (direction == "")
            {
                var distanToLeft = CalculateDistance(origin, left);
                var distanToRight = CalculateDistance(origin, right);
                if (distanToLeft < distanToRight) { direction = "left"; }
                else { direction = "right"; }
            }

        }
        if (direction == "left") { finalTargetEdge = left; }
        else if (direction == "right") { finalTargetEdge = right; }
        // Console.WriteLine();

        if (finalTargetEdge == (-1, -1))
        {
            // Console.WriteLine("No path. Both edges are not near the border");
            Environment.Exit(0);
        }
        else
        {
            // Console.WriteLine($"Final Target Edge: {finalTargetEdge}");
            // Console.WriteLine();
        }

        // Find the best detour point
        var detourCandidates = GetAdjacentTiles(finalTargetEdge);
        var filteredDetourCandidates = new List<(int, int)>();
        // if (detourCandidates.Count > 0) { Console.Write("Detour Candidate: "); }

        foreach (var item in detourCandidates)
        {
            if (origin == item)
            {
                // Console.WriteLine();
                // Console.WriteLine($"Same with start: {item}");
                // Console.WriteLine();
                continue;
            }
            if (Closed.Contains(item) || Obstacles.Contains(item) || !IsReachableDirectly(origin, item).Item1 ) { continue; }
            filteredDetourCandidates.Add(item);
            // Console.Write(" " + item);
        }
        // Console.WriteLine();
        var (leftMostTarget, rightMostTarget) = FindExtremeAngleTargets(origin, filteredDetourCandidates, true);
        // Console.WriteLine("direction: " + direction);
        var bestDetourPoint = (-1, -1);

        if(PreviousOrigin != (-1, -1) && bestDetourPoint == (-1, -1)) {
            var distanceFromPreviousOriginToLeft = CalculateDistance(PreviousOrigin, leftMostTarget);
            var distanceFromPreviousOriginToRight = CalculateDistance(PreviousOrigin, rightMostTarget);
            if (distanceFromPreviousOriginToLeft < distanceFromPreviousOriginToRight) { bestDetourPoint = rightMostTarget; }
            else if (distanceFromPreviousOriginToLeft > distanceFromPreviousOriginToRight) { bestDetourPoint = leftMostTarget; }
        }

        if(bestDetourPoint == (-1, -1)) {
            if (direction == "left") { bestDetourPoint = leftMostTarget; }
            else if (direction == "right") { bestDetourPoint = rightMostTarget; }
        }

        // stopwatch.Stop(); ElapsedTime += stopwatch.ElapsedMilliseconds;
        // Console.WriteLine();
        // Console.WriteLine($"Elapsed time: {stopwatch.ElapsedMilliseconds}ms");
        // Console.WriteLine($"bestDetourPoint: {bestDetourPoint}");

        return bestDetourPoint;
    }
    public static List<(int X, int Y)> SimplifyWaypoints(List<(int X, int Y)> waypoints)
    {
        if (waypoints.Count < 2) { return waypoints; }
        var simplified = new List<(int X, int Y)> { waypoints[0] };
        for (int i = 1; i < waypoints.Count - 1; i++)
        {
            var (prevX, prevY) = waypoints[i - 1];
            var curr = waypoints[i];
            var (nextX, nextY) = waypoints[i + 1];
            if ((curr.X - prevX) * (nextY - curr.Y) != (curr.Y - prevY) * (nextX - curr.X))
            {
                simplified.Add(curr);
            }
        }
        simplified.Add(waypoints[^1]);
        return simplified;
    }
    public List<(int X, int Y)> GetPathCoordinates(List<(int X, int Y)> waypoints)
    {
        var allCoordinates = new List<(int X, int Y)>();
        if (waypoints == null || waypoints.Count < 2) { return allCoordinates; }
        for (int i = 0; i < waypoints.Count - 1; i++)
        {
            var from = waypoints[i];
            var to = waypoints[i + 1];
            var (isReachable, _, path) = IsReachableDirectly(from, to);
            if (!isReachable) { return allCoordinates; }
            if (allCoordinates.Count > 0 && path.Count > 0 && allCoordinates[^1] == path[0]) { path.RemoveAt(0); }
            allCoordinates.AddRange(path);
        }
        return allCoordinates;
    }
    public (List<(int X, int Y)>, PathCase) GetWaypoints()
    {
        PathCase pathCase = ProceedBasicSearch();
        // Console.WriteLine("pathCase: " + pathCase);
        if (pathCase == PathCase.reachableDirectly) { return ([Start, Goal], pathCase); }
        else if (pathCase == PathCase.reachableAlongOneDirection) { return (Waypoints, pathCase); }
        else if (pathCase == PathCase.notReachable)
        {
            if (HitObstacle == (-1, -1))
            {
                // Console.WriteLine("HitObstacle is not set");
                return (Waypoints, pathCase);
            }
            var bestDetourPoint = ProceedNormalSearch();

            if (bestDetourPoint == (-1, -1)) { return ([], PathCase.noPath); }

            Waypoints = [bestDetourPoint];
            return (Waypoints, pathCase);
        }
        // Console.WriteLine("pathCase: " + pathCase);
        return (Waypoints, pathCase);
    }
    public (List<(int X, int Y)>, PathCase) FindPath()
    {
        var (waypoints, pathCase) = GetWaypoints();
        if (waypoints.Count > 0)
        {
            waypoints.Insert(0, Start);
            if (pathCase == PathCase.notReachable)
            {
                waypoints = SimplifyWaypoints(waypoints);
            }
            else if (pathCase == PathCase.reachableAlongOneDirection)
            {
                waypoints.Add(Goal);
                waypoints = SimplifyWaypoints(waypoints);
            }
            else if (pathCase == PathCase.reachableDirectly)
            {
                waypoints = [Start, Goal];
            }
            else
            {
                waypoints.Add(Goal);
            }

            if (DrawMap)
            {
                var pathCoordinates = GetPathCoordinates(waypoints);
                // if (pathCoordinates.Count > 0) { Console.Write("Path: "); }
                // foreach (var point in pathCoordinates) { Console.Write(" " + point); }
                // Console.WriteLine();

                // Console.WriteLine();
                // Console.WriteLine("Start: " + Start + " -> Goal: " + Goal);
                // Console.WriteLine();
                // if (waypoints.Count > 0) { Console.Write("waypoint: "); }
                // foreach (var item in waypoints)
                // {
                    // Console.Write(" " + item);
                // }
                // Console.WriteLine();

                var detourPoint = new List<(int X, int Y)>();
                foreach (var item in waypoints)
                {
                    if (item != Start && item != Goal) { detourPoint.Add(item); }
                }

                DisplayMap(pathCoordinates, detourPoint, null);
            }
        }
        else
        {
            Console.WriteLine("No path");
        }

        return (waypoints, pathCase);
    }

}

class Program
{
    static void Main()
    {
        var obstacles = new HashSet<(int X, int Y)>() {
                                                                                                            (8, 0),
                                                                                                            (8, 1),
                                    (2, 2),         (3, 2),                                                 (8, 2),
                                    (2, 3),                                                                 (8, 3),
                                                                                                (7, 4),     (8, 4),
                                                    (3, 5),     (4, 5), (5, 5),     (6, 5),     (7, 5),
                                                    (3, 6),
                                                    (3, 7),             (5, 7),     (6, 7),
                                                    (3, 8),             (5, 8),     (6, 8),
                                                    (3, 9),             (5, 9),                 (7, 9),
                                                    (3, 10),                (5, 10),            (7, 10),
                        (1, 11),    (2, 11)
        };

        // (50, 50)
        obstacles.Add((57,47));
        obstacles.Add((57,46));
        obstacles.Add((56,46));
        obstacles.Add((55,46));
        obstacles.Add((55,46));
        obstacles.Add((55,45));
        obstacles.Add((54,45));
        obstacles.Add((53,45));
        obstacles.Add((52,45));
        obstacles.Add((51,45));
        obstacles.Add((50,45));
        obstacles.Add((49,45));
        obstacles.Add((48,45));
        obstacles.Add((47,45));
        obstacles.Add((46,45));
        obstacles.Add((45,45));
        obstacles.Add((44,45));
        obstacles.Add((44,46));
        obstacles.Add((43,46));
        obstacles.Add((43,47));
        obstacles.Add((43,48));
        obstacles.Add((42,48));
        obstacles.Add((42,49));
        obstacles.Add((41,49));
        obstacles.Add((41,50));
        obstacles.Add((41,51));
        obstacles.Add((41,52));
        obstacles.Add((41,53));
        obstacles.Add((41,54));
        obstacles.Add((42,54));
        obstacles.Add((42,55));
        obstacles.Add((42,56));
        obstacles.Add((43,56));
        obstacles.Add((43,57));
        obstacles.Add((44,57));
        obstacles.Add((44,58));
        obstacles.Add((45,58));
        obstacles.Add((45,59));
        obstacles.Add((46,59));
        obstacles.Add((47,59));

        // (100, 100)
        obstacles.Add((111,101));
        obstacles.Add((112,101));
        obstacles.Add((112,100));
        obstacles.Add((113,100));
        obstacles.Add((113,99));
        obstacles.Add((113,98));
        obstacles.Add((113,97));
        obstacles.Add((113,96));
        obstacles.Add((113,95));
        obstacles.Add((112,95));
        obstacles.Add((112,94));
        obstacles.Add((111,95));
        obstacles.Add((111,94));
        obstacles.Add((110,94));
        obstacles.Add((110,93));
        obstacles.Add((110,93));
        obstacles.Add((109,93));
        obstacles.Add((108,93));
        obstacles.Add((107,93));
        obstacles.Add((107,92));
        obstacles.Add((106,92));
        obstacles.Add((105,92));
        obstacles.Add((104,92));
        obstacles.Add((103,92));
        obstacles.Add((103,93));
        obstacles.Add((102,93));
        obstacles.Add((101,93));
        obstacles.Add((100,93));
        obstacles.Add((99,93));
        obstacles.Add((99,94));
        obstacles.Add((98,94));
        obstacles.Add((97,94));
        obstacles.Add((97,95));
        obstacles.Add((96,95));
        obstacles.Add((95,95));
        obstacles.Add((95,96));
        obstacles.Add((94,96));
        obstacles.Add((94,97));
        obstacles.Add((94,98));
        obstacles.Add((93,98));
        obstacles.Add((93,99));
        obstacles.Add((93,100));
        obstacles.Add((93,101));
        obstacles.Add((92,101));
        obstacles.Add((92,102));
        obstacles.Add((92,103));
        obstacles.Add((92,104));
        obstacles.Add((93,104));
        obstacles.Add((93,105));
        obstacles.Add((93,106));
        obstacles.Add((94,105));
        obstacles.Add((94,106));
        obstacles.Add((94,107));
        obstacles.Add((94,108));
        obstacles.Add((95,108));
        obstacles.Add((96,108));
        obstacles.Add((96,109));
        obstacles.Add((97,109));
        obstacles.Add((98,109));
        obstacles.Add((98,110));
        obstacles.Add((99,110));

        // (150, 150)
        obstacles.Add((146, 162));
        obstacles.Add((146, 161));
        obstacles.Add((145, 161));
        obstacles.Add((145, 160));
        obstacles.Add((145, 159));
        obstacles.Add((145, 158));
        obstacles.Add((144, 158));
        obstacles.Add((144, 157));
        obstacles.Add((144, 156));
        obstacles.Add((143, 156));
        obstacles.Add((143, 155));
        obstacles.Add((143, 154));
        obstacles.Add((142, 154));
        obstacles.Add((142, 153));
        obstacles.Add((142, 152));
        obstacles.Add((141, 152));
        obstacles.Add((141, 151));
        obstacles.Add((141, 150));
        obstacles.Add((141, 149));
        obstacles.Add((140, 149));
        obstacles.Add((140, 148));
        obstacles.Add((140, 147));
        obstacles.Add((140, 146));
        obstacles.Add((140, 145));
        obstacles.Add((140, 144));
        obstacles.Add((141, 144));
        obstacles.Add((141, 143));
        obstacles.Add((142, 143));
        obstacles.Add((142, 142));
        obstacles.Add((142, 141));
        obstacles.Add((143, 141));
        obstacles.Add((143, 140));
        obstacles.Add((144, 140));
        obstacles.Add((145, 140));
        obstacles.Add((145, 139));
        obstacles.Add((146, 139));
        obstacles.Add((147, 139));
        obstacles.Add((147, 138));
        obstacles.Add((148, 138));
        obstacles.Add((149, 138));
        obstacles.Add((150, 138));
        obstacles.Add((151, 138));
        obstacles.Add((152, 137));
        obstacles.Add((153, 137));
        obstacles.Add((154, 137));
        obstacles.Add((155, 137));
        obstacles.Add((156, 137));
        obstacles.Add((157, 137));
        obstacles.Add((158, 137));
        obstacles.Add((159, 137));
        obstacles.Add((160, 137));
        obstacles.Add((160, 138));
        obstacles.Add((161, 138));
        obstacles.Add((162, 138));
        obstacles.Add((163, 138));
        obstacles.Add((163, 139));
        obstacles.Add((164, 139));
        obstacles.Add((165, 139));
        obstacles.Add((165, 140));
        obstacles.Add((166, 140));
        obstacles.Add((167, 140));
        obstacles.Add((167, 141));
        obstacles.Add((168, 141));
        obstacles.Add((168, 142));
        obstacles.Add((168, 143));
        obstacles.Add((168, 144));
        obstacles.Add((168, 145));
        obstacles.Add((168, 146));
        obstacles.Add((169, 146));
        obstacles.Add((169, 147));
        obstacles.Add((169, 148));
        obstacles.Add((169, 149));
        obstacles.Add((169, 150));
        obstacles.Add((168, 150));
        obstacles.Add((168, 151));
        obstacles.Add((167, 151));
        obstacles.Add((167, 152));
        obstacles.Add((166, 152));
        obstacles.Add((166, 153));
        obstacles.Add((165, 153));
        obstacles.Add((164, 153));
        obstacles.Add((164, 154));
        obstacles.Add((163, 154));


        var sightRange = 15;
        var moveRange = 15;
        (int X, int Y) tileRangeStart = (0, 0);
        (int X, int Y) tileRangeEnd = (1920, 1080);
        var closed = new HashSet<(int X, int Y)>();
        var hitObstacle = (-1, -1);
        var start = (-1, -1); var goal = (-1, -1);
        start = (7, 3); goal = (9, 4); closed = [];
        start = (7, 3); goal = (50, 50); closed = [];
        start = (7, 3); goal = (100, 100); closed = [];
        start = (7, 3); goal = (150, 150); closed = [];
        // start = (7, 3); goal = (200, 200); closed = [];
        // start = (7, 3); goal = (250, 250); closed = [];
        // start = (7, 3); goal = (300, 300); closed = [];
        // start = (3, 4); goal = (9, 4); closed = [(7,3)]; hitObstacle = (8,3);
        // start = (2, 4); goal = (9, 4); closed = [(2,4),(3,4),(7,3),]; hitObstacle = (7,4);
        // start = (0, 10); goal = (9, 4); closed = [(2,4),(3,4),(7,3),]; hitObstacle = (7,4);
        // start = (0, 11); goal = (9, 4); closed = [(0,10),(2,4),(3,4),(7,3),]; hitObstacle = (3,9);
        // start = (0, 12); goal = (9, 4); closed = [(0,11),(0,10),(2,4),(3,4),(7,3),]; hitObstacle = (1,11);

        var pathFinder = new PathFinder(
            tileRangeStart: tileRangeStart,
            tileRangeEnd: tileRangeEnd,
            start: start,
            goal: goal,
            sightRange: sightRange,
            moveRange: moveRange,
            previousHitObstacle: hitObstacle,
            closed: closed,
            obstacles: obstacles,
            drawMap: false
        );

        Stopwatch stopwatch = new();
        stopwatch.Start();
        var (finalPath, pathCase) = FindPathRecursive(pathFinder, []);
        stopwatch.Stop();
        Console.WriteLine("Start: " + start + " -> Goal: " + goal);
        Console.WriteLine("BW Total Elapsed time: " + stopwatch.ElapsedMilliseconds + "ms");
        Console.WriteLine("pathCase: " + pathCase);
        foreach (var item in finalPath)
        {
            Console.WriteLine("finalPath: " + item);
        }

        stopwatch.Restart();
        var astarPath = AStar(start, goal, tileRangeStart, tileRangeEnd, obstacles);
        stopwatch.Stop();
        Console.WriteLine("A* Total Elapsed time: " + stopwatch.ElapsedMilliseconds + "ms");
        // foreach (var item in astarPath)
        // {
        //     Console.Write(" " + item);
        // }
    }

    public static (List<(int X, int Y)>, PathCase) FindPathRecursive(PathFinder pathFinder, List<(int X, int Y)> currentPath)
    {
        var (path, pathCase) = pathFinder.FindPath();
        if (path.Count == 0) { return (currentPath, pathCase); }
        currentPath.AddRange(path);
        var lastPath = path.Last();
        if (lastPath == pathFinder.Goal) { return (currentPath, pathCase); }

        currentPath.RemoveAt(currentPath.Count - 1);

        pathFinder.Start = lastPath;
        pathFinder.PreviousHitObstacle = pathFinder.HitObstacle;
        foreach(var item in path) { pathFinder.Closed.Add(item); }

        return FindPathRecursive(pathFinder, currentPath);
    }

    static List<(int X, int Y)> AStar(
        (int X, int Y) start,
        (int X, int Y) goal,
        (int X, int Y) tileRangeStart,
        (int X, int Y) tileRangeEnd,
        HashSet<(int X, int Y)> obstacles)
    {
        var openSet = new PriorityQueue<(int X, int Y), int>();
        var cameFrom = new Dictionary<(int X, int Y), (int X, int Y)>();
        var gScore = new Dictionary<(int X, int Y), int>();
        var fScore = new Dictionary<(int X, int Y), int>();

        gScore[start] = 0;
        fScore[start] = Heuristic(start, goal);
        openSet.Enqueue(start, fScore[start]);

        while (openSet.Count > 0)
        {
            var current = openSet.Dequeue();

            if (current == goal)
            {
                return ReconstructPath(cameFrom, current);
            }

            foreach (var neighbor in GetNeighbors(current, tileRangeStart, tileRangeEnd, obstacles))
            {
                var tentativeGScore = gScore[current] + 1;

                if (!gScore.ContainsKey(neighbor) || tentativeGScore < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = gScore[neighbor] + Heuristic(neighbor, goal);

                    openSet.Enqueue(neighbor, fScore[neighbor]);
                }
            }
        }

        return [];
    }


    static int Heuristic((int X, int Y) a, (int X, int Y) b)
    {
        return Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);
    }

    static IEnumerable<(int X, int Y)> GetNeighbors((int X, int Y) node, (int X, int Y) tileRangeStart, (int X, int Y) tileRangeEnd, HashSet<(int X, int Y)> obstacles)
    {
        var directions = new List<(int X, int Y)> { (0, 1), (1, 0), (0, -1), (-1, 0) };
        foreach (var dir in directions)
        {
            (int X, int Y) neighbor = (node.X + dir.X, node.Y + dir.Y);
            if (neighbor.X >= tileRangeStart.X && neighbor.X <= tileRangeEnd.X &&
                neighbor.Y >= tileRangeStart.Y && neighbor.Y <= tileRangeEnd.Y &&
                !obstacles.Contains(neighbor))
            {
                yield return neighbor;
            }
        }
    }

    static List<(int X, int Y)> ReconstructPath(Dictionary<(int X, int Y), (int X, int Y)> cameFrom, (int X, int Y) current)
    {
        var path = new List<(int X, int Y)> { current };
        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            path.Add(current);
        }
        path.Reverse();
        return path;
    }
}