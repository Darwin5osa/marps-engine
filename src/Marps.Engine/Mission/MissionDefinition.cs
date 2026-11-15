using System;
using System.Collections.Generic;
using System.Linq;
using Marps.Engine.Geometry;

namespace Marps.Engine.Mission
{
    /// <summary>
    /// A mission place. Build it freely; once it is passed to a <see cref="MissionDefinition"/> the mission keeps
    /// its own frozen copy, so the places the engine navigates to can never be changed from outside
    /// (setting a property of a frozen place throws <see cref="InvalidOperationException"/>).
    /// </summary>
    public sealed class Waypoint
    {
        private string _id, _type, _name, _reason;
        private Point2 _position;
        private double _projectionX, _projectionY;
        private bool _frozen;

        public string Id { get => _id; set { Check(); _id = value; } }
        /// <summary>Place type: "base", "science", "antenna" or "hazard". Comes from mission.json.</summary>
        public string Type { get => _type; set { Check(); _type = value; } }
        public string Name { get => _name; set { Check(); _name = value; } }
        public string Reason { get => _reason; set { Check(); _reason = value; } }
        /// <summary>Position in local meters (x east, z north).</summary>
        public Point2 Position { get => _position; set { Check(); _position = value; } }
        /// <summary>Projection X, Y, as they appear in mission.json.</summary>
        public double ProjectionX { get => _projectionX; set { Check(); _projectionX = value; } }
        public double ProjectionY { get => _projectionY; set { Check(); _projectionY = value; } }
        /// <summary>True if it is part of the EVA objective sequence (set by the mission).</summary>
        public bool IsObjective { get => _isObjective; internal set { Check(); _isObjective = value; } }
        private bool _isObjective;

        /// <summary>True once the place belongs to a mission and can no longer change.</summary>
        public bool IsFrozen => _frozen;

        internal Waypoint FrozenCopy(bool isObjective) => new Waypoint
        {
            _id = _id, _type = _type, _name = _name, _reason = _reason, _position = _position,
            _projectionX = _projectionX, _projectionY = _projectionY,
            _isObjective = isObjective, _frozen = true,
        };

        private void Check()
        {
            if (_frozen)
                throw new InvalidOperationException("This place belongs to a mission and cannot be changed; build a new MissionDefinition instead.");
        }
    }

    /// <summary>
    /// Full mission: ordered objectives, points of interest and hazard zones. Immutable: it copies and freezes
    /// the places and zones it is given, so later changes to the caller's objects do not reach it.
    /// </summary>
    public sealed class MissionDefinition
    {
        private readonly Dictionary<string, Waypoint> _byId;
        private readonly HashSet<string> _objectiveSet;

        public MissionDefinition(string name, string objective, IEnumerable<Waypoint> waypoints,
            IEnumerable<string> objectiveOrder, IEnumerable<HazardZone> hazardZones)
        {
            if (waypoints == null) throw new ArgumentNullException(nameof(waypoints));
            if (objectiveOrder == null) throw new ArgumentNullException(nameof(objectiveOrder));

            Name = name;
            Objective = objective;
            // Step sequence: ids may repeat (the base appears on departure and on return).
            var order = objectiveOrder.ToList();
            _objectiveSet = new HashSet<string>(order.Where(id => id != null), StringComparer.Ordinal);

            var places = new List<Waypoint>();
            _byId = new Dictionary<string, Waypoint>(StringComparer.Ordinal);
            foreach (var wp in waypoints)
            {
                if (wp == null) throw new ArgumentException("The places list contains a null place.", nameof(waypoints));
                if (string.IsNullOrEmpty(wp.Id)) throw new ArgumentException("Every place needs a non-empty id.", nameof(waypoints));
                if (_byId.ContainsKey(wp.Id)) throw new ArgumentException($"Place id '{wp.Id}' is used twice.", nameof(waypoints));
                if (!double.IsFinite(wp.Position.X) || !double.IsFinite(wp.Position.Z))
                    throw new ArgumentException($"Place '{wp.Id}' has a non-finite position.", nameof(waypoints));
                var copy = wp.FrozenCopy(_objectiveSet.Contains(wp.Id));
                places.Add(copy);
                _byId.Add(copy.Id, copy);
            }

            foreach (var id in order)
            {
                if (id == null || !_byId.ContainsKey(id))
                    throw new ArgumentException($"Objective '{id}' does not exist in the places list.", nameof(objectiveOrder));
            }

            var zones = new List<HazardZone>();
            foreach (var z in hazardZones ?? Enumerable.Empty<HazardZone>())
            {
                if (z == null) throw new ArgumentException("The hazard zones list contains a null zone.", nameof(hazardZones));
                zones.Add(z.FrozenCopy());
            }

            Waypoints = places.AsReadOnly();
            ObjectiveIds = order.AsReadOnly();
            HazardZones = zones.AsReadOnly();
            Objectives = order.Select(id => _byId[id]).ToList().AsReadOnly();
            Antennas = places.Where(w => w.Type == "antenna" || w.Type == "base").ToList().AsReadOnly();
        }

        public string Name { get; }
        public string Objective { get; }
        public IReadOnlyList<Waypoint> Waypoints { get; }
        /// <summary>Mission steps in order, by place id (B, P1, P2, P3, P4, B).</summary>
        public IReadOnlyList<string> ObjectiveIds { get; }
        public IReadOnlyList<HazardZone> HazardZones { get; }

        /// <summary>The place with this id, or null. O(1), no allocations (used on every engine frame).</summary>
        public Waypoint Find(string id) => id != null && _byId.TryGetValue(id, out var wp) ? wp : null;

        /// <summary>True if the id is one of the mission steps.</summary>
        public bool HasObjective(string id) => id != null && _objectiveSet.Contains(id);

        /// <summary>The places of each step, in order.</summary>
        public IReadOnlyList<Waypoint> Objectives { get; }

        /// <summary>Points that can emit/receive signal (base and radio masts).</summary>
        public IReadOnlyList<Waypoint> Antennas { get; }
    }
}
