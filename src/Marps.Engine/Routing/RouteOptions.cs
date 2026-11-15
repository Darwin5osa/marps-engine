using System;

namespace Marps.Engine.Routing
{
    /// <summary>Route search settings. The engine copies them when it is created: later changes have no effect.</summary>
    public sealed class RouteOptions
    {
        /// <summary>Maximum slope allowed in each step between neighboring nodes (degrees).</summary>
        public double MaxSlopeDeg { get; set; } = 22.0;

        /// <summary>
        /// Cost multiplier when entering a rough zone (hazard 3).
        /// ASSUMPTION: 1.0 = no extra penalty. Must be tuned with terrain judgment.
        /// </summary>
        public double RoughFactor { get; set; } = 1.0;

        /// <summary>
        /// Weight applied to the A* heuristic. 1.0 gives optimal routes; values above 1.0 search fewer
        /// nodes but may return routes up to roughly this factor more expensive than the optimum.
        /// </summary>
        public double HeuristicWeight { get; set; } = 1.25;

        /// <summary>A validated copy (throws on values the search cannot work with).</summary>
        internal RouteOptions Snapshot()
        {
            if (!(MaxSlopeDeg > 0 && MaxSlopeDeg < 90))
                throw new ArgumentOutOfRangeException(nameof(MaxSlopeDeg), "Must be between 0 and 90 degrees (exclusive).");
            if (!(RoughFactor > 0) || double.IsInfinity(RoughFactor))
                throw new ArgumentOutOfRangeException(nameof(RoughFactor), "Must be a positive finite number.");
            if (!(HeuristicWeight >= 0) || double.IsInfinity(HeuristicWeight))
                throw new ArgumentOutOfRangeException(nameof(HeuristicWeight), "Must be a finite number >= 0.");
            return new RouteOptions { MaxSlopeDeg = MaxSlopeDeg, RoughFactor = RoughFactor, HeuristicWeight = HeuristicWeight };
        }
    }
}
