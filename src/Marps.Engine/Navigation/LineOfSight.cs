using System;
using Marps.Engine.Geometry;
using Marps.Engine.Terrain;

namespace Marps.Engine.Navigation
{
    /// <summary>Line of sight over the terrain: the ray between two heights may not cross the ground.</summary>
    public static class LineOfSight
    {
        public static bool Clear(TerrainMap map, Point2 from, double fromHeightAboveGroundM,
            Point2 to, double toHeightAboveGroundM)
        {
            double h1 = map.ElevationAt(from) + fromHeightAboveGroundM;
            double h2 = map.ElevationAt(to) + toHeightAboveGroundM;
            double length = from.DistanceTo(to);
            // Clamp in floating point before the cast: casting a huge or infinite value to int saturates
            // to int.MaxValue and would make this loop run about two billion times.
            // Two samples per cell along the map diagonal is more than enough to see any terrain crest.
            double maxSteps = 2.0 * (map.Width + map.Height);
            double wanted = Math.Max(2.0, Math.Ceiling(length / map.Cell));
            int steps = (int)Math.Min(wanted, maxSteps);

            for (int i = 1; i < steps; i++)
            {
                double t = (double)i / steps;
                var p = new Point2(from.X + (to.X - from.X) * t, from.Z + (to.Z - from.Z) * t);
                double ray = h1 + (h2 - h1) * t;
                if (map.ElevationAt(p) > ray) return false;
            }
            return true;
        }
    }
}
