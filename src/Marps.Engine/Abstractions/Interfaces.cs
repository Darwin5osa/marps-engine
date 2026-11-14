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
    /// Everything a frontend (Unity, web, hardware) can ask the navigation engine. Frontends should depend on
    /// this interface, not on <see cref="NavigationEngine"/>. Implementations are thread-safe.
    /// </summary>
    public interface INavigationEngine : IDisposable
    {
        TerrainMap Map { get; }
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
