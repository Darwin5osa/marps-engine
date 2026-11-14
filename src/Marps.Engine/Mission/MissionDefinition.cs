using System;
using System.Collections.Generic;
using System.Linq;
using Marps.Engine.Geometry;

namespace Marps.Engine.Mission
{
    public sealed class Waypoint
    {
        public string Id { get; set; }
        /// <summary>Place type: "base", "science", "antenna" or "hazard". Comes from mission.json.</summary>
        public string Type { get; set; }
        public string Name { get; set; }
        public string Reason { get; set; }
        /// <summary>Position in local meters (x east, z north).</summary>
        public Point2 Position { get; set; }
        /// <summary>Projection X, Y, as they appear in mission.json.</summary>
        public double ProjectionX { get; set; }
        public double ProjectionY { get; set; }
        /// <summary>True if it is part of the EVA objective sequence.</summary>
        public bool IsObjective { get; set; }
    }

    /// <summary>Full mission: ordered objectives, points of interest and hazard zones.</summary>
    public sealed class MissionDefinition
    {
        public MissionDefinition(string name, string objective, IEnumerable<Waypoint> waypoints,
            IEnumerable<string> objectiveOrder, IEnumerable<HazardZone> hazardZones)
        {
            Name = name;
            Objective = objective;
            Waypoints = waypoints?.ToList() ?? throw new ArgumentNullException(nameof(waypoints));
            // Step sequence: ids may repeat (the base appears on departure and on return).
            ObjectiveIds = objectiveOrder?.ToList() ?? throw new ArgumentNullException(nameof(objectiveOrder));
            HazardZones = hazardZones?.ToList() ?? new List<HazardZone>();

            foreach (var id in ObjectiveIds)
            {
                if (!Waypoints.Any(w => w.Id == id))
                    throw new ArgumentException($"Objective '{id}' does not exist in the places list.");
            }
            foreach (var wp in Waypoints)
                wp.IsObjective = ObjectiveIds.Contains(wp.Id);
        }

        public string Name { get; }
        public string Objective { get; }
        public IReadOnlyList<Waypoint> Waypoints { get; }
        /// <summary>Mission steps in order, by place id (B, P1, P2, P3, P4, B).</summary>
        public IReadOnlyList<string> ObjectiveIds { get; }
        public IReadOnlyList<HazardZone> HazardZones { get; }

        public Waypoint Find(string id) => Waypoints.FirstOrDefault(w => w.Id == id);

        public IEnumerable<Waypoint> Objectives => ObjectiveIds.Select(Find);

        /// <summary>Points that can emit/receive signal (base and radio masts).</summary>
        public IEnumerable<Waypoint> Antennas => Waypoints.Where(w => w.Type == "antenna" || w.Type == "base");
    }
}
