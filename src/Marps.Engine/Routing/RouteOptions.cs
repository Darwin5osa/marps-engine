namespace Marps.Engine.Routing
{
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
    }
}
