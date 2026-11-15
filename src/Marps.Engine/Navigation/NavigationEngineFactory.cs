using Marps.Engine.Abstractions;
using Marps.Engine.Mission;
using Marps.Engine.Routing;
using Marps.Engine.Terrain;

namespace Marps.Engine.Navigation
{
    /// <summary>
    /// The only way to create a navigation engine. It hands back the <see cref="INavigationEngine"/> interface;
    /// the implementation is internal, so frontends (Unity, the web harness, hardware) can only use the engine
    /// through its interfaces.
    /// </summary>
    public static class NavigationEngineFactory
    {
        /// <summary>
        /// Creates an engine over a map and a mission (both immutable). The options are copied and validated
        /// (an invalid value throws <see cref="System.ArgumentOutOfRangeException"/>); changing them afterwards
        /// has no effect. Dispose the engine when done (required with <see cref="NavigationOptions.BackgroundReplanning"/>).
        /// </summary>
        public static INavigationEngine Create(TerrainMap map, MissionDefinition mission,
            NavigationOptions options = null, WalkingModel model = null, RouteOptions routeOptions = null) =>
            new NavigationEngine(map, mission, options, model, routeOptions);
    }
}
