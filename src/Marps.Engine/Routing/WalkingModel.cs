using System;

namespace Marps.Engine.Routing
{
    /// <summary>
    /// Walking time in suit, in minutes, per step:
    ///   t = FlatMinPerM * horizontal_distance + ClimbMinPerM * ascent + DescentMinPerM * descent
    /// Coefficients fitted by least squares to the 5 reference legs (reference_routes.json, with the 22° rule):
    /// maximum error 0.11 min. This is a fit to 5 data points (3 parameters): it must be validated with real data.
    /// Immutable.
    /// </summary>
    public sealed class WalkingModel
    {
        public static readonly WalkingModel Fitted = new WalkingModel(0.013937, 0.090788, 0.042108);

        public WalkingModel(double flatMinPerM, double climbMinPerM, double descentMinPerM)
        {
            // A zero or negative coefficient would make steps free or "pay back" time, which breaks the search.
            if (!(flatMinPerM > 0) || double.IsInfinity(flatMinPerM))
                throw new ArgumentOutOfRangeException(nameof(flatMinPerM), "Must be a positive finite number.");
            if (!(climbMinPerM >= 0) || double.IsInfinity(climbMinPerM))
                throw new ArgumentOutOfRangeException(nameof(climbMinPerM), "Must be a finite number >= 0.");
            if (!(descentMinPerM >= 0) || double.IsInfinity(descentMinPerM))
                throw new ArgumentOutOfRangeException(nameof(descentMinPerM), "Must be a finite number >= 0.");
            FlatMinPerM = flatMinPerM;
            ClimbMinPerM = climbMinPerM;
            DescentMinPerM = descentMinPerM;
        }

        public double FlatMinPerM { get; }
        public double ClimbMinPerM { get; }
        public double DescentMinPerM { get; }

        /// <summary>Cost (minutes) of a step. <paramref name="dh"/> = destination height minus origin height.</summary>
        public double EdgeMinutes(double horizontalM, double dh)
        {
            return dh >= 0
                ? FlatMinPerM * horizontalM + ClimbMinPerM * dh
                : FlatMinPerM * horizontalM - DescentMinPerM * dh;
        }

        /// <summary>Admissible lower bound for A*: never overestimates.</summary>
        public double LowerBoundMinutes(double euclideanM) => FlatMinPerM * euclideanM;

        /// <summary>
        /// Horizontal walking speed in m/s that this model assumes on a slope of <paramref name="slopeDeg"/>
        /// degrees, going up (<paramref name="uphill"/> = true) or down. It is the inverse of
        /// <see cref="EdgeMinutes"/>, so a player moved at this speed takes exactly the estimated time.
        /// The sign of the slope is ignored (the direction comes from <paramref name="uphill"/>); NaN counts as flat;
        /// 90° or more gives 0. On flat ground: 1 / (0.013937 · 60) ≈ 1.2 m/s.
        /// </summary>
        public double SpeedAt(double slopeDeg, bool uphill)
        {
            double s = double.IsNaN(slopeDeg) ? 0 : Math.Abs(slopeDeg);
            if (s >= 90) return 0;
            double tan = Math.Tan(s * Math.PI / 180.0);
            double minPerHorizontalM = FlatMinPerM + (uphill ? ClimbMinPerM : DescentMinPerM) * tan;
            return 1.0 / (minPerHorizontalM * 60.0);
        }
    }
}
