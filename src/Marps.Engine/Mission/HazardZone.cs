using System;
using Marps.Engine.Geometry;

namespace Marps.Engine.Mission
{
    /// <summary>Hazard zone (from hazards.json) with its box in local meters.</summary>
    public sealed class HazardZone
    {
        public string Type { get; set; }
        public double AreaM2 { get; set; }
        public Point2 Center { get; set; }
        public Point2 BoxMin { get; set; }
        public Point2 BoxMax { get; set; }
        public double MaxSlopeDeg { get; set; }
        public double FractionOver15 { get; set; }

        /// <summary>Distance from a point to the zone's box (0 if inside).</summary>
        public double DistanceTo(Point2 p)
        {
            double dx = Math.Max(Math.Max(BoxMin.X - p.X, 0), p.X - BoxMax.X);
            double dz = Math.Max(Math.Max(BoxMin.Z - p.Z, 0), p.Z - BoxMax.Z);
            return Math.Sqrt(dx * dx + dz * dz);
        }

        /// <summary>Severity: "high" if the slope exceeds 22°, otherwise "moderate".</summary>
        public string Severity => MaxSlopeDeg > 22.0 ? "high" : "moderate";
    }
}
