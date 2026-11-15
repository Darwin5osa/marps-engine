using System.Collections.Generic;
using System.Linq;
using Marps.Engine.Abstractions;
using Marps.Engine.Geometry;
using Marps.Engine.Mission;
using Marps.Engine.Navigation;
using Marps.Engine.Routing;
using Marps.Engine.Terrain;
using Marps.Engine.Tests.Helpers;
using Xunit;
using static Marps.Engine.Tests.StopOrderTests;

namespace Marps.Engine.Tests
{
    /// <summary>The engine features are reachable through <see cref="INavigationEngine"/> alone.</summary>
    public class EngineInterfaceTests
    {
        private static INavigationEngine NewEngine() => new NavigationEngine(TestTerrain.Flat(60, 2.0), SquareMission());

        [Fact]
        public void ExposesMapMissionAndBase()
        {
            using var engine = NewEngine();
            Assert.NotNull(engine.Map);
            Assert.Equal("square", engine.Mission.Name);
            Assert.Equal("B", engine.Base.Id);
        }

        [Fact]
        public void ReorderStops_ChangesTheActiveStop()
        {
            var engine = NewEngine();
            engine.Update(At(10, 10));
            var resp = engine.ReorderStops(At(10, 10), new[] { "P3", "P2", "P1" });
            Assert.Equal("P3", resp.NextWaypoint.Id);
            Assert.True(resp.Route.Replanned);
            Assert.Equal(new[] { "B", "P3", "P2", "P1", "B" }, resp.Objectives.Select(o => o.Id).ToArray());
        }

        [Fact]
        public void StartAndCancelEmergencyReturn()
        {
            var engine = NewEngine();
            engine.Update(At(10, 10));
            engine.Update(At(100, 10));

            var sos = engine.StartEmergencyReturn(At(100, 50));
            Assert.True(sos.Emergency.Active);
            Assert.Equal("B", sos.Route.Target);
            Assert.True(sos.Route.Found);

            var resumed = engine.CancelEmergencyReturn(At(100, 50));
            Assert.Null(resumed.Emergency);
            Assert.Equal("P2", resumed.NextWaypoint.Id);
        }

        [Fact]
        public void ConvenienceMethods_DoNotChangeTheCallersRequest()
        {
            var engine = NewEngine();
            var req = At(10, 10);
            engine.StartEmergencyReturn(req);
            engine.ReorderStops(req, new[] { "P2" });
            Assert.False(req.EmergencyReturn);
            Assert.Null(req.StopOrder);

            // A plain Update with the same object does not re-trigger anything; the session keeps its state.
            var again = engine.Update(req);
            Assert.True(again.Emergency.Active);
        }

        [Fact]
        public void ConvenienceMethods_RejectNull()
        {
            var engine = NewEngine();
            Assert.Throws<System.ArgumentNullException>(() => engine.StartEmergencyReturn(null));
            Assert.Throws<System.ArgumentNullException>(() => engine.CancelEmergencyReturn(null));
            Assert.Throws<System.ArgumentNullException>(() => engine.ReorderStops(At(10, 10), null));
        }

        [Fact]
        public void PlanRouteItineraryAndTrip_AgreeOnTheSameLegs()
        {
            var engine = NewEngine();
            var from = new Point2(10, 10);
            RouteResult route = engine.PlanRoute(from, new Point2(100, 10));
            List<ItineraryLeg> itinerary = engine.PlanItinerary(from, new[] { "P1", "P2" });
            TripPlan trip = engine.PlanTrip(new TripRequest { X = 10, Z = 10, Stops = { TripStop.Place("P1") }, Destination = TripStop.Place("P2") });

            Assert.Equal(route.HorizontalLengthM, itinerary[0].Route.HorizontalLengthM, 6);
            Assert.Equal(itinerary[0].Route.HorizontalLengthM, trip.Legs[0].LengthM, 6);
            Assert.Equal(itinerary[1].Route.HorizontalLengthM, trip.Legs[1].LengthM, 6);
        }

        [Fact]
        public void Service_WorksWithAnyEngineImplementation()
        {
            var fake = new FakeEngine();
            var service = new NavigationService(fake, new FixedSensor(), null);
            var resp = service.Step();
            Assert.Equal(1, fake.Updates);
            Assert.Equal("fake", resp.SessionId);
        }

        private sealed class FixedSensor : IAstronautSensor
        {
            public NavigationRequest Read() => At(1, 2, "fake");
        }

        /// <summary>A frontend test double: proves nothing in the service needs the concrete engine.</summary>
        private sealed class FakeEngine : INavigationEngine
        {
            public int Updates;
            public TerrainMap Map => null;
            public MissionDefinition Mission => null;
            public Waypoint Base => null;
            public NavigationResponse Update(NavigationRequest request)
            {
                Updates++;
                return new NavigationResponse { SessionId = request.SessionId };
            }
            public NavigationResponse ReorderStops(NavigationRequest current, IList<string> stopOrder) => Update(current);
            public NavigationResponse StartEmergencyReturn(NavigationRequest current) => Update(current);
            public NavigationResponse CancelEmergencyReturn(NavigationRequest current) => Update(current);
            public RouteResult PlanRoute(Point2 from, Point2 to) => RouteResult.NotFound(0, 0);
            public List<ItineraryLeg> PlanItinerary(Point2 from, IEnumerable<string> placeIds) => new List<ItineraryLeg>();
            public TripPlan PlanTrip(TripRequest request) => new TripPlan();
            public TripPlan PlanTrip(Point2 current, IEnumerable<Point2> stops, Point2 destination) => new TripPlan();
            public CostField GetCostField(string placeId) => null;
            public CostField GetCostField(Point2 target) => null;
            public double SpeedAt(double slopeDeg, bool uphill) => 1.0;
            public void Dispose() { }
        }
    }
}
