using System;
using System.Collections.Generic;
using Marps.Engine.Geometry;
using Marps.Engine.Mission;
using Marps.Engine.Navigation;
using Marps.Engine.Routing;
using Marps.Engine.Terrain;

namespace Marps.Engine.Abstractions
{
    /// <summary>
    /// Everything a frontend (Unity, web, hardware) can ask the navigation engine. Get one from
    /// <see cref="NavigationEngineFactory.Create"/>: the implementation is internal, so this interface is the only
    /// way in. Implementations are thread-safe.
    /// </summary>
    public interface INavigationEngine : IDisposable
    {
        /// <summary>The terrain (immutable: readable per node or as copies).</summary>
        TerrainMap Map { get; }

        /// <summary>The mission (immutable: its places and zones are frozen).</summary>
        MissionDefinition Mission { get; }

        /// <summary>The base used for emergency return, or null if the mission has none.</summary>
        Waypoint Base { get; }

        /// <summary>One navigation step for a session: progress, route to the active target, alerts and layers.</summary>
        NavigationResponse Update(NavigationRequest request);

        /// <summary>
        /// Reorders the stops still ahead in the session of <paramref name="current"/> and returns the updated
        /// step (route recalculated to the new active stop). Same rules as <see cref="NavigationRequest.StopOrder"/>.
        /// </summary>
        NavigationResponse ReorderStops(NavigationRequest current, IList<string> stopOrder);

        /// <summary>Emergency button: returns the step with the route straight to the base.</summary>
        NavigationResponse StartEmergencyReturn(NavigationRequest current);

        /// <summary>Leaves emergency mode and returns the step with the mission resumed.</summary>
        NavigationResponse CancelEmergencyReturn(NavigationRequest current);

        /// <summary>Safe route between two points, without a session.</summary>
        RouteResult PlanRoute(Point2 from, Point2 to);

        /// <summary>Every leg from <paramref name="from"/> through the given mission places, in order.</summary>
        List<ItineraryLeg> PlanItinerary(Point2 from, IEnumerable<string> placeIds);

        /// <summary>
        /// Route from the current position through the stops (mission places or free points) to the destination.
        /// Bad input is reported in <see cref="TripPlan.Error"/> instead of throwing.
        /// </summary>
        TripPlan PlanTrip(TripRequest request);

        /// <summary>Same as <see cref="PlanTrip(TripRequest)"/> with free points.</summary>
        TripPlan PlanTrip(Point2 current, IEnumerable<Point2> stops, Point2 destination);

        /// <summary>
        /// Walking minutes from every node to a mission place (e.g. <c>Base.Id</c>), for the O₂ point of no return
        /// and the walk-back contour. Built on the first call for each place (a whole-map search: about 0.7 s on a
        /// desktop PC for the real 1025x1025 grid, several times more on a Quest 2, so call it off the frame thread,
        /// e.g. at startup) and cached; building it never blocks <see cref="Update"/>.
        /// Throws <see cref="ArgumentException"/> for an unknown place.
        /// </summary>
        CostField GetCostField(string placeId);

        /// <summary>
        /// Same as <see cref="GetCostField(string)"/> for a free point inside the map (e.g. a shelter). Not cached.
        /// Throws <see cref="ArgumentException"/> for a non-finite point or one outside the map.
        /// </summary>
        CostField GetCostField(Point2 target);

        /// <summary>
        /// Horizontal walking speed (m/s) the engine's time model assumes on a slope, going up or down, so the
        /// player can walk at the speed the time estimates use. About 1.2 m/s on flat ground.
        /// </summary>
        double SpeedAt(double slopeDeg, bool uphill);
    }

    /// <summary>
    /// Source of the astronaut state: Quest headset (Unity), web simulator, physical sensors, etc.
    /// The engine does not know where the data comes from.
    /// </summary>
    public interface IAstronautSensor
    {
        NavigationRequest Read();
    }

    /// <summary>HUD destination: Unity, a test viewer, a physical panel, etc.</summary>
    public interface IHudSink
    {
        void Publish(NavigationResponse response);
    }

    /// <summary>Space weather provider (DONKI or another source).</summary>
    public interface ISpaceWeatherProvider
    {
        SpaceWeather Current();
    }

    /// <summary>Terrain map provider (processed files, streaming, etc.).</summary>
    public interface ITerrainProvider
    {
        TerrainMap Load();
    }
}
