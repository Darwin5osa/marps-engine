using System.Collections.Generic;
using Marps.Engine.Geometry;

namespace Marps.Engine.Navigation
{
    /// <summary>
    /// A point of a trip: either a mission place by <see cref="Id"/>, or free coordinates
    /// (<see cref="X"/>, <see cref="Z"/>, local meters). When both are given, the id wins.
    /// </summary>
    public sealed class TripStop
    {
        public string Id { get; set; }
        public double? X { get; set; }
        public double? Z { get; set; }

        public static TripStop Place(string id) => new TripStop { Id = id };
        public static TripStop At(double x, double z) => new TripStop { X = x, Z = z };
        public static TripStop At(Point2 p) => new TripStop { X = p.X, Z = p.Z };
    }

    /// <summary>Input of <see cref="Abstractions.INavigationEngine.PlanTrip(TripRequest)"/>.</summary>
    public sealed class TripRequest
    {
        /// <summary>Current position (local meters).</summary>
        public double X { get; set; }
        public double Z { get; set; }
        /// <summary>Intermediate stops, visited in this order. May be empty.</summary>
        public List<TripStop> Stops { get; set; } = new List<TripStop>();
        /// <summary>Final destination. Required.</summary>
        public TripStop Destination { get; set; }
    }

    /// <summary>Planned trip: one leg per stop plus the final leg, and the whole route joined.</summary>
    public sealed class TripPlan
    {
        /// <summary>True when every leg has a safe route.</summary>
        public bool Found { get; set; }
        /// <summary>Why the request could not be planned (bad input); null when it was planned.</summary>
        public string Error { get; set; }
        public double LengthM { get; set; }
        public double EstimatedMinutes { get; set; }
        public double AscentM { get; set; }
        public double DescentM { get; set; }
        public double MaxSlopeDeg { get; set; }
        public List<TripLeg> Legs { get; set; } = new List<TripLeg>();
        /// <summary>The whole route from the current position to the destination, simplified for display.</summary>
        public List<Point2> Points { get; set; } = new List<Point2>();
        public double ComputeMs { get; set; }
    }

    public sealed class TripLeg
    {
        /// <summary>Leg number (0-based). The last leg ends at the destination.</summary>
        public int Index { get; set; }
        /// <summary>Id of the place the leg ends at, or null for a free point.</summary>
        public string ToId { get; set; }
        public double FromX { get; set; }
        public double FromZ { get; set; }
        public double ToX { get; set; }
        public double ToZ { get; set; }
        public bool Found { get; set; }
        public double LengthM { get; set; }
        public double EstimatedMinutes { get; set; }
        public double AscentM { get; set; }
        public double DescentM { get; set; }
        public double MaxSlopeDeg { get; set; }
        /// <summary>Route of this leg, simplified for display. Empty when no route was found.</summary>
        public List<Point2> Points { get; set; } = new List<Point2>();
    }
}
