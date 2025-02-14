using System.Drawing;

namespace bw_path_finding;
public static class Methods
{
    public static HashSet<(int X, int Y)> GatherDebugMepData(int topLeftX, int topLeftY, string imageFileFullPath)
    {
        string currentDirectory = AppContext.BaseDirectory;
        var imagePath = Path.Combine(currentDirectory, imageFileFullPath);
        Console.WriteLine($"Loading image from: {imagePath}");

        int tileSize = 16;
        HashSet<Color> validColors =
        [
            Color.FromArgb(238, 237, 235), // EEEDEB
            Color.FromArgb(255, 106, 0)    // FF6A00
        ];

        var obstaclesTiles = new List<(int X, int Y)>();

        using (Bitmap image = new Bitmap(imagePath))
        {
            int width = image.Width;
            int height = image.Height;

            for (int imgY = 0; imgY < height; imgY++)
            {
                for (int imgX = 0; imgX < width; imgX++)
                {
                    Color pixelColor = image.GetPixel(imgX, imgY);
                    if(imgX == 4 && imgY == 6) {
                        Console.WriteLine(pixelColor);
                    }
                    int worldX = topLeftX + imgX * tileSize;
                    int worldY = topLeftY + imgY * tileSize;
                    int posX = worldX / tileSize;
                    int posY = worldY / tileSize;
                    if(posX == 1124 && posY == 822) {
                        Console.WriteLine("");
                    }
                    if (!validColors.Contains(pixelColor))
                    {
                        obstaclesTiles.Add((worldX, worldY));
                    }
                }
            }
        }

        var obstacles = new HashSet<(int X, int Y)>();
        foreach (var (x, y) in obstaclesTiles)
        {
            for (var i = 0; i < tileSize; i++)
            {
                for (var j = 0; j < tileSize; j++)
                {
                    obstacles.Add((x + i, y + j));
                }
            }
        }

        Console.WriteLine("obstacles.Count: " + obstacles.Count);
        return obstacles;
    }
    public static void RearrangeTileRange(
        (int X, int Y) Start, 
        (int X, int Y) End, 
        ref (int X, int Y) TileRangeStart, 
        ref (int X, int Y) TileRangeEnd
    ) {
        var plusMinus = 16 * 2;
        var limitX = 1920 * 16;
        var limitY = 1080 * 16;

        var minX = Math.Min(Start.X, End.X) - plusMinus;
        var maxX = Math.Max(Start.X, End.X) + plusMinus;
        var minY = Math.Min(Start.Y, End.Y) - plusMinus;
        var maxY = Math.Max(Start.Y, End.Y) + plusMinus;

        if (minX < 0) { minX = 0; }
        if (minY < 0) { minY = 0; }

        if (maxX > limitX) { maxX = limitX; }
        if (maxY > limitY) { maxY = limitY; }

        TileRangeStart = (minX, minY);
        TileRangeEnd = (maxX, maxY);
    }
}