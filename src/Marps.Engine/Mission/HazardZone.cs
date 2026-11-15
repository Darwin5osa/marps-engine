using System;
using Marps.Engine.Geometry;

namespace Marps.Engine.Mission
{
    /// <summary>
    /// Hazard zone (from hazards.json) with its box in local meters. Like <see cref="Waypoint"/>, a mission keeps
    /// a frozen copy: setting a property of a zone that belongs to a mission throws.
    /// </summary>
    public sealed class HazardZone
    {
        private string _type;
        private double _areaM2, _maxSlopeDeg, _fractionOver15;
        private Point2 _center, _boxMin, _boxMax;
        private bool _frozen;

        public string Type { get => _type; set { Check(); _type = value; } }
        public double AreaM2 { get => _areaM2; set { Check(); _areaM2 = value; } }
        public Point2 Center { get => _center; set { Check(); _center = value; } }
        public Point2 BoxMin { get => _boxMin; set { Check(); _boxMin = value; } }
        public Point2 BoxMax { get => _boxMax; set { Check(); _boxMax = value; } }
        public double MaxSlopeDeg { get => _maxSlopeDeg; set { Check(); _maxSlopeDeg = value; } }
        public double FractionOver15 { get => _fractionOver15; set { Check(); _fractionOver15 = value; } }

        /// <summary>True once the zone belongs to a mission and can no longer change.</summary>
        public bool IsFrozen => _frozen;

        /// <summary>Distance from a point to the zone's box (0 if inside).</summary>
        public double DistanceTo(Point2 p)
        {
            double dx = Math.Max(Math.Max(BoxMin.X - p.X, 0), p.X - BoxMax.X);
            double dz = Math.Max(Math.Max(BoxMin.Z - p.Z, 0), p.Z - BoxMax.Z);
            return Math.Sqrt(dx * dx + dz * dz);
        }

        /// <summary>Severity: "high" if the slope exceeds 22°, otherwise "moderate".</summary>
        public string Severity => MaxSlopeDeg > 22.0 ? "high" : "moderate";

        internal HazardZone FrozenCopy() => new HazardZone
        {
            _type = _type, _areaM2 = _areaM2, _center = _center, _boxMin = _boxMin, _boxMax = _boxMax,
            _maxSlopeDeg = _maxSlopeDeg, _fractionOver15 = _fractionOver15, _frozen = true,
        };

        private void Check()
        {
            if (_frozen)
                throw new InvalidOperationException("This zone belongs to a mission and cannot be changed; build a new MissionDefinition instead.");
        }
    }
}
