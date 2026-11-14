using System.Collections.Generic;
using Marps.Engine.Geometry;

namespace Marps.Engine.Navigation
{
    // ===================== ENTRADA =====================

    /// <summary>Astronaut state coming from the sensor. Positions in local meters (x east, z north).</summary>
    public sealed class NavigationRequest
    {
        public string SessionId { get; set; } = "default";
        public double X { get; set; }
        public double Z { get; set; }
        /// <summary>Look direction: 0 = north, 90 = east.</summary>
        public double HeadingDeg { get; set; }
        public double PitchDeg { get; set; }
        public double MissionTimeS { get; set; }
        /// <summary>Objective chosen by the astronaut; if null, the engine uses the next pending one.</summary>
        public string ActiveWaypointId { get; set; }
        /// <summary>Forces a route recalculation even if nothing changed.</summary>
        public bool ForceReplan { get; set; }
        public SpaceWeather SpaceWeather { get; set; }
    }

    public sealed class SpaceWeather
    {
        public bool StormActive { get; set; }
        public string Level { get; set; }
        public string Source { get; set; }
    }

    // ===================== OUTPUT =====================

    public sealed class NavigationResponse
    {
        public string SessionId { get; set; }
        public double MissionTimeS { get; set; }
        public bool MissionComplete { get; set; }
        public PositionInfo Position { get; set; }
        public RouteInfo Route { get; set; }
        public NextWaypointInfo NextWaypoint { get; set; }
        public List<ObjectiveInfo> Objectives { get; set; } = new List<ObjectiveInfo>();
        public List<HazardInfo> Hazards { get; set; } = new List<HazardInfo>();
        public List<AlertInfo> Alerts { get; set; } = new List<AlertInfo>();
        public LayerInfo Layers { get; set; }
        public double ComputeMs { get; set; }
    }

    public sealed class PositionInfo
    {
        public double X { get; set; }
        public double Z { get; set; }
        public double ElevationM { get; set; }
        public bool Inside { get; set; }
    }

    public sealed class RouteInfo
    {
        public string Target { get; set; }
        public bool Found { get; set; }
        public bool Replanned { get; set; }
        public double LengthM { get; set; }
        public double RemainingM { get; set; }
        public double EstimatedMinutes { get; set; }
        public double AscentM { get; set; }
        public double DescentM { get; set; }
        public double MaxSlopeDeg { get; set; }
        /// <summary>Polyline of the remaining leg (from the current position), simplified to 0.5 m.</summary>
        public List<Point2> Points { get; set; } = new List<Point2>();
    }

    public sealed class NextWaypointInfo
    {
        public string Id { get; set; }
        public string Type { get; set; }
        public string Name { get; set; }
        public double X { get; set; }
        public double Z { get; set; }
        /// <summary>Straight-line distance, in meters.</summary>
        public double DistanceM { get; set; }
        /// <summary>Rumbo absoluto (0 = norte).</summary>
        public double BearingDeg { get; set; }
        /// <summary>Bearing relative to the look direction: negative = left, positive = right.</summary>
        public double RelativeBearingDeg { get; set; }
    }

    public sealed class ObjectiveInfo
    {
        /// <summary>Position of the step in the mission sequence (0-based).</summary>
        public int Step { get; set; }
        public string Id { get; set; }
        public string Name { get; set; }
        /// <summary>done, active o pending.</summary>
        public string Status { get; set; }
        public double X { get; set; }
        public double Z { get; set; }
    }

    public sealed class HazardInfo
    {
        public string Type { get; set; }
        public string Severity { get; set; }
        public double DistanceM { get; set; }
        public double CenterX { get; set; }
        public double CenterZ { get; set; }
        public double BoxMinX { get; set; }
        public double BoxMinZ { get; set; }
        public double BoxMaxX { get; set; }
        public double BoxMaxZ { get; set; }
        public double MaxSlopeDeg { get; set; }
        public double AreaM2 { get; set; }
    }

    public sealed class AlertInfo
    {
        /// <summary>HAZARD_AHEAD, SIGNAL_LOSS, SIGNAL_LIMITED, SOLAR_STORM, OFF_MAP, NO_ROUTE.</summary>
        public string Code { get; set; }
        /// <summary>info, warning o danger.</summary>
        public string Severity { get; set; }
        public string Message { get; set; }
    }

    public sealed class LayerInfo
    {
        public double SlopeDeg { get; set; }
        /// <summary>0 walkable, 1 slope 15-22°, 2 slope &gt;22°, 3 rough zone.</summary>
        public int HazardClass { get; set; }
        public double ElevationM { get; set; }
        public int VisibleAntennas { get; set; }
        public List<AntennaInfo> Antennas { get; set; } = new List<AntennaInfo>();
    }

    public sealed class AntennaInfo
    {
        public string Id { get; set; }
        public bool Visible { get; set; }
    }
}
