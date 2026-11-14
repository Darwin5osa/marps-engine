using System;
using System.Collections.Generic;
using System.Linq;
using Marps.Engine.Abstractions;
using Marps.Engine.Geometry;
using Marps.Engine.Json;
using Marps.Engine.Navigation;
using Marps.Engine.Terrain;
using Marps.Engine.Tests.Helpers;
using Xunit;
using static Marps.Engine.Tests.StopOrderTests;

namespace Marps.Engine.Tests
{
    public class TripPlanningTests
    {
        // Used only through the interface, as a frontend would.
        private static INavigationEngine NewEngine(TerrainMap map = null) =>
            new NavigationEngine(map ?? TestTerrain.Flat(60, 2.0), SquareMission());

        private static Point2 Last(List<Point2> points) => points[points.Count - 1];

        [Fact]
        public void PlacesAsStops_OneLegPerStopPlusDestination()
        {
            var engine = NewEngine();
            var plan = engine.PlanTrip(new TripRequest
            {
                X = 10, Z = 10,
                Stops = { TripStop.Place("P1"), TripStop.Place("P2") },
                Destination = TripStop.Place("B"),
            });

            Assert.Null(plan.Error);
            Assert.True(plan.Found);
            Assert.Equal(3, plan.Legs.Count);
            Assert.Equal(new[] { "P1", "P2", "B" }, plan.Legs.Select(l => l.ToId).ToArray());
            Assert.Equal(new[] { 0, 1, 2 }, plan.Legs.Select(l => l.Index).ToArray());

            // Each leg starts where the previous one ended.
            Assert.Equal(10, plan.Legs[0].FromX);
            Assert.Equal(100, plan.Legs[1].FromX);
            Assert.Equal(10, plan.Legs[1].FromZ);
            Assert.Equal(100, plan.Legs[2].FromZ);

            // Totals are the sum of the legs; the B -> P1 -> P2 -> B triangle is about 90 + 90 + 127 m.
            Assert.Equal(plan.Legs.Sum(l => l.LengthM), plan.LengthM, 6);
            Assert.Equal(plan.Legs.Sum(l => l.EstimatedMinutes), plan.EstimatedMinutes, 6);
            Assert.InRange(plan.LengthM, 300, 330);
        }

        [Fact]
        public void JoinedRoute_GoesFromCurrentPointToDestination()
        {
            var engine = NewEngine();
            var plan = engine.PlanTrip(new Point2(10, 10), new[] { new Point2(100, 100) }, new Point2(10, 100));

            Assert.True(plan.Found);
            Assert.True(plan.Points[0].DistanceTo(new Point2(10, 10)) < 3.0);
            Assert.True(Last(plan.Points).DistanceTo(new Point2(10, 100)) < 3.0);
            Assert.Contains(plan.Points, p => p.DistanceTo(new Point2(100, 100)) < 3.0);   // passes the stop
            Assert.Null(plan.Legs[0].ToId);                                                // free points have no id
            Assert.Equal(100, plan.Legs[0].ToX);
        }

        [Fact]
        public void NoStops_IsADirectRoute()
        {
            var engine = NewEngine();
            var plan = engine.PlanTrip(new TripRequest { X = 10, Z = 10, Destination = TripStop.Place("P1") });
            Assert.True(plan.Found);
            Assert.Single(plan.Legs);
            Assert.InRange(plan.LengthM, 89, 95);

            var direct = engine.PlanRoute(new Point2(10, 10), new Point2(100, 10));
            Assert.Equal(direct.HorizontalLengthM, plan.LengthM, 6);
        }

        [Fact]
        public void PlacesAndFreePoints_CanBeMixed()
        {
            var engine = NewEngine();
            var plan = engine.PlanTrip(new TripRequest
            {
                X = 10, Z = 10,
                Stops = { TripStop.At(55, 55), TripStop.Place("P3") },
                Destination = TripStop.At(100, 100),
            });
            Assert.True(plan.Found);
            Assert.Equal(new string[] { null, "P3", null }, plan.Legs.Select(l => l.ToId).ToArray());
        }

        [Fact]
        public void IdWinsOverCoordinates()
        {
            var engine = NewEngine();
            var plan = engine.PlanTrip(new TripRequest
            {
                X = 10, Z = 10,
                Destination = new TripStop { Id = "P2", X = 1, Z = 1 },
            });
            Assert.Equal(100, plan.Legs[0].ToX);
            Assert.Equal(100, plan.Legs[0].ToZ);
        }

        [Fact]
        public void BadInput_ReturnsAnError_WithoutSearching()
        {
            var engine = NewEngine();

            var noDest = engine.PlanTrip(new TripRequest { X = 10, Z = 10 });
            Assert.False(noDest.Found);
            Assert.Contains("destination", noDest.Error);
            Assert.Empty(noDest.Legs);

            var unknown = engine.PlanTrip(new TripRequest { X = 10, Z = 10, Stops = { TripStop.Place("NOPE") }, Destination = TripStop.Place("B") });
            Assert.Contains("Stop 1", unknown.Error);
            Assert.Contains("NOPE", unknown.Error);

            var noCoords = engine.PlanTrip(new TripRequest { X = 10, Z = 10, Destination = new TripStop { X = 5 } });
            Assert.Contains("Destination", noCoords.Error);

            var nan = engine.PlanTrip(new TripRequest { X = double.NaN, Z = 10, Destination = TripStop.Place("B") });
            Assert.NotNull(nan.Error);

            var nullStop = engine.PlanTrip(new TripRequest { X = 10, Z = 10, Stops = { null }, Destination = TripStop.Place("B") });
            Assert.Contains("Stop 1", nullStop.Error);

            Assert.Throws<ArgumentNullException>(() => engine.PlanTrip(null));
        }

        [Fact]
        public void UnreachableStop_MarksThatLeg_ButStillPlansTheRest()
        {
            // A wall of blocked terrain (hazard 2) around the point (100, 100).
            var map = TestTerrain.WithHazard(TestTerrain.Flat(60, 2.0), (c, r) =>
            {
                double x = c * 2.0, z = (59 - r) * 2.0;
                double d = Math.Max(Math.Abs(x - 100), Math.Abs(z - 100));
                return (byte)(d >= 8 && d <= 10 ? 2 : 0);
            });
            var engine = NewEngine(map);

            var plan = engine.PlanTrip(new Point2(10, 10), new[] { new Point2(100, 100) }, new Point2(10, 100));

            Assert.False(plan.Found);
            Assert.Null(plan.Error);                        // the input was fine; the terrain is the problem
            Assert.False(plan.Legs[0].Found);
            Assert.Empty(plan.Legs[0].Points);
            Assert.Empty(plan.Points);                      // no joined route when a leg is missing
            Assert.Equal(2, plan.Legs.Count);               // the leg after the unreachable stop is still reported
            Assert.False(plan.Legs[1].Found);               // it starts inside the wall too
        }

        [Fact]
        public void Trip_DoesNotTouchSessions()
        {
            var engine = NewEngine();
            var before = engine.Update(At(10, 10));
            engine.PlanTrip(new TripRequest { X = 10, Z = 10, Stops = { TripStop.Place("P3") }, Destination = TripStop.Place("P2") });
            var after = engine.Update(At(10, 10));
            Assert.Equal(before.NextWaypoint.Id, after.NextWaypoint.Id);
            Assert.False(after.Route.Replanned);
        }

        [Fact]
        public void Json_UsesTheContractNames()
        {
            var req = MarpsJson.Deserialize<TripRequest>(
                "{\"x\":10,\"z\":10,\"stops\":[{\"id\":\"P1\"},{\"x\":55,\"z\":55}],\"destination\":{\"id\":\"B\"}}");
            Assert.Equal("P1", req.Stops[0].Id);
            Assert.Equal(55, req.Stops[1].X);

            var json = MarpsJson.Serialize(NewEngine().PlanTrip(req));
            Assert.Contains("\"found\":true", json);
            Assert.Contains("\"legs\":[{\"index\":0,\"toId\":\"P1\"", json);
            Assert.DoesNotContain("\"error\"", json);       // null fields are omitted
        }
    }
}
