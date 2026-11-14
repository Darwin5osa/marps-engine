using Marps.Engine.Geometry;
using Marps.Engine.Routing;

namespace Marps.Engine.Navigation
{
    /// <summary>One leg of an itinerary planned with <see cref="NavigationEngine.PlanItinerary"/>.</summary>
    public sealed class ItineraryLeg
    {
        /// <summary>Id of the place this leg ends at.</summary>
        public string TargetId { get; set; }
        /// <summary>Where the leg starts (the previous place, or the start point for the first leg).</summary>
        public Point2 From { get; set; }
        public RouteResult Route { get; set; }
    }
}
