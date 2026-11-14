using System.Text.Json.Serialization;
using Marps.Engine.Geometry;

namespace Marps.Engine.Terrain
{
    /// <summary>
    /// Map metadata (terrain.json). Projection coordinates (X, Y) are in meters
    /// of the Mars equirectangular projection; the engine works in local meters.
    /// </summary>
    public sealed class TerrainMetadata
    {
        [JsonPropertyName("width")]
        public int Width { get; set; }

        [JsonPropertyName("height")]
        public int Height { get; set; }

        /// <summary>Projection X of the southwest corner.</summary>
        [JsonPropertyName("originX")]
        public double OriginX { get; set; }

        /// <summary>Projection Y of the southwest corner.</summary>
        [JsonPropertyName("originY")]
        public double OriginY { get; set; }

        /// <summary>Side of the square in meters (edge to edge of the grid).</summary>
        [JsonPropertyName("sideM")]
        public double SideM { get; set; }

        [JsonPropertyName("zMin")]
        public double ZMin { get; set; }

        [JsonPropertyName("zMax")]
        public double ZMax { get; set; }

        [JsonPropertyName("projection")]
        public string Projection { get; set; }

        [JsonPropertyName("heightsFile")]
        public string HeightsFile { get; set; }

        [JsonPropertyName("hazardFile")]
        public string HazardFile { get; set; }

        /// <summary>Distance between neighboring nodes. With 1025 nodes over 2000 m: 1.953125 m (same as Unity).</summary>
        [JsonIgnore]
        public double CellM => SideM / (Width - 1);

        /// <summary>Converts projection X, Y to local meters (x east, z north, SW origin).</summary>
        public Point2 ProjectionToLocal(double x, double y) => new Point2(x - OriginX, y - OriginY);

        /// <summary>Converts local meters to projection X, Y.</summary>
        public (double X, double Y) LocalToProjection(Point2 p) => (p.X + OriginX, p.Z + OriginY);
    }
}
