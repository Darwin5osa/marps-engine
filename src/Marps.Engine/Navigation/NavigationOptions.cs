namespace Marps.Engine.Navigation
{
    /// <summary>
    /// Engine parameters. Those marked ASSUMPTION do not come from the data and must be validated.
    /// </summary>
    public sealed class NavigationOptions
    {
        /// <summary>Distance at which a target counts as reached.</summary>
        public double ReachRadiusM { get; set; } = 10.0;

        /// <summary>If the astronaut moves farther than this from the route, it is recalculated.</summary>
        public double OffRouteM { get; set; } = 15.0;

        /// <summary>Radius for listing nearby hazard zones.</summary>
        public double HazardRadiusM { get; set; } = 150.0;

        /// <summary>Distance looked ahead for the terrain alert.</summary>
        public double LookAheadM { get; set; } = 40.0;

        /// <summary>Slope from which a warning is raised (the hard rule is the one in RouteOptions).</summary>
        public double WarnSlopeDeg { get; set; } = 15.0;

        /// <summary>ASSUMPTION: height of the astronaut's eyes above the ground.</summary>
        public double EyeHeightM { get; set; } = 1.7;

        /// <summary>ASSUMPTION: height of the radio masts above the ground. Not in the data.</summary>
        public double AntennaHeightM { get; set; } = 3.0;

        /// <summary>Maximum route slope (must match RouteOptions.MaxSlopeDeg).</summary>
        public double MaxSlopeDeg { get; set; } = 22.0;

        /// <summary>Simplification tolerance for the polyline sent to the HUD.</summary>
        public double PolylineToleranceM { get; set; } = 0.5;

        /// <summary>
        /// Maximum number of astronaut sessions kept in memory. When a new session would exceed this,
        /// the least recently used session is discarded (its progress and cached route are lost).
        /// </summary>
        public int MaxSessions { get; set; } = 4096;
    }
}
