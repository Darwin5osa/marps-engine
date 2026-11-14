using System;
using System.Text.Json.Serialization;

namespace Marps.Engine.Geometry
{
    /// <summary>
    /// Point in local meters (Unity convention): X = east, Z = north,
    /// origin at the southwest corner of the map.
    /// </summary>
    public struct Point2
    {
        [JsonPropertyName("x")]
        public double X { get; set; }

        [JsonPropertyName("z")]
        public double Z { get; set; }

        public Point2(double x, double z)
        {
            X = x;
            Z = z;
        }

        public double DistanceTo(Point2 other)
        {
            double dx = X - other.X;
            double dz = Z - other.Z;
            return Math.Sqrt(dx * dx + dz * dz);
        }

        public override string ToString() => $"({X:0.00}, {Z:0.00})";
    }
}
