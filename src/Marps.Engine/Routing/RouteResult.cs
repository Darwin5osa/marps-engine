using System.Collections.Generic;
using Marps.Engine.Geometry;

namespace Marps.Engine.Routing
{
    public sealed class RouteResult
    {
        public bool Found { get; set; }
        public List<Point2> Points { get; set; } = new List<Point2>();

        /// <summary>Horizontal length of the trip in meters (excluding vertical climbs).</summary>
        public double HorizontalLengthM { get; set; }
        public double AscentM { get; set; }
        public double DescentM { get; set; }

        /// <summary>Estimated time in suit, in minutes (cost optimized by A*).</summary>
        public double EstimatedMinutes { get; set; }

        /// <summary>Maximum slope between consecutive nodes of the trip (degrees).</summary>
        public double MaxEdgeSlopeDeg { get; set; }

        /// <summary>Nodes expanded by A* (a measure of effort).</summary>
        public int Expansions { get; set; }

        public double ComputeMs { get; set; }

        public static RouteResult NotFound(int expansions, double computeMs) =>
            new RouteResult { Found = false, Expansions = expansions, ComputeMs = computeMs };
    }
}
