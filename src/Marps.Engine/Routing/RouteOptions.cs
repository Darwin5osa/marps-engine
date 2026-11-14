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
    }
}
