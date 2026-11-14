using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using Marps.Engine.Geometry;
using Marps.Engine.Mission;
using Marps.Engine.Navigation;
using Marps.Engine.Terrain;
using Marps.Engine.Tests.Helpers;
using Xunit;

namespace Marps.Engine.Tests
{
    public class StopOrderTests
    {
        // Flat 118 m x 118 m map. Mission B -> P1 -> P2 -> P3 -> B, with an antenna that is not a stop.
        internal static MissionDefinition SquareMission() => new MissionDefinition("square", "test",
            new[]
            {
                new Waypoint { Id = "B", Type = "base", Name = "Base", Position = new Point2(10, 10) },
                new Waypoint { Id = "P1", Type = "science", Name = "P1", Position = new Point2(100, 10) },
                new Waypoint { Id = "P2", Type = "science", Name = "P2", Position = new Point2(100, 100) },
                new Waypoint { Id = "P3", Type = "science", Name = "P3", Position = new Point2(10, 100) },
                new Waypoint { Id = "A1", Type = "antenna", Name = "A1", Position = new Point2(60, 60) },
            },
            new[] { "B", "P1", "P2", "P3", "B" },
            new HazardZone[0]);

        internal static NavigationRequest At(double x, double z, string session = "s1") =>
            new NavigationRequest { SessionId = session, X = x, Z = z };

        private static NavigationRequest Reorder(double x, double z, params string[] order)
        {
            var req = At(x, z);
            req.StopOrder = order.ToList();
            return req;
        }

        private static string[] Ids(NavigationResponse r) => r.Objectives.Select(o => o.Id).ToArray();

        private static NavigationEngine NewEngine(NavigationOptions options = null) =>
            new NavigationEngine(TestTerrain.Flat(60, 2.0), SquareMission(), options);

        [Fact]
        public void NewSession_FollowsTheMissionOrder()
        {
            var engine = NewEngine();
            var resp = engine.Update(At(10, 10));
            Assert.Equal(new[] { "B", "P1", "P2", "P3", "B" }, Ids(resp));
            Assert.Equal("P1", resp.NextWaypoint.Id);
        }

        [Fact]
        public void FullPermutation_ChangesActiveStopAndRecalculatesRoute()
        {
            var engine = NewEngine();
            engine.Update(At(10, 10));                                   // leaves the base, route to P1

            var resp = engine.Update(Reorder(10, 10, "P3", "P2", "P1"));

            Assert.Equal(new[] { "B", "P3", "P2", "P1", "B" }, Ids(resp));
            Assert.Equal("P3", resp.NextWaypoint.Id);
            Assert.Equal("P3", resp.Route.Target);
            Assert.True(resp.Route.Found);
            Assert.True(resp.Route.Replanned);
            Assert.Equal(new[] { "done", "active", "pending", "pending", "pending" }, resp.Objectives.Select(o => o.Status).ToArray());
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "INVALID_STOP_ORDER");

            // The route really goes to the new stop: it ends at P3.
            var end = resp.Route.Points[resp.Route.Points.Count - 1];
            Assert.True(end.DistanceTo(new Point2(10, 100)) < 3.0);
        }

        [Fact]
        public void PartialList_PutsListedStopsFirst_AndKeepsTheRestInOrder()
        {
            var engine = NewEngine();
            var resp = engine.Update(Reorder(10, 10, "P2"));
            Assert.Equal(new[] { "B", "P2", "P1", "P3", "B" }, Ids(resp));
            Assert.Equal("P2", resp.NextWaypoint.Id);
        }

        [Fact]
        public void CompletedStops_AreIgnored_SoTheFullListCanBeResent()
        {
            var engine = NewEngine();
            engine.Update(At(10, 10));
            engine.Update(At(100, 10));                                  // P1 reached

            var resp = engine.Update(Reorder(100, 10, "P1", "P3", "P2"));

            Assert.DoesNotContain(resp.Alerts, a => a.Code == "INVALID_STOP_ORDER");
            Assert.Equal(new[] { "B", "P1", "P3", "P2", "B" }, Ids(resp));
            Assert.Equal("done", resp.Objectives[1].Status);
            Assert.Equal("P3", resp.NextWaypoint.Id);
        }

        [Fact]
        public void DepartureAndReturnBase_StayFixed()
        {
            var engine = NewEngine();
            var resp = engine.Update(Reorder(10, 10, "B", "P3"));
            Assert.Equal(new[] { "B", "P3", "P1", "P2", "B" }, Ids(resp));
        }

        [Theory]
        [InlineData("X")]       // unknown place
        [InlineData("A1")]      // a place that is not a mission stop
        [InlineData("")]        // empty id
        public void InvalidId_IsRejected_AndOrderIsUnchanged(string bad)
        {
            var engine = NewEngine();
            var resp = engine.Update(Reorder(10, 10, "P3", bad));
            Assert.Contains(resp.Alerts, a => a.Code == "INVALID_STOP_ORDER" && a.Severity == "warning");
            Assert.Equal(new[] { "B", "P1", "P2", "P3", "B" }, Ids(resp));
            Assert.Equal("P1", resp.NextWaypoint.Id);
        }

        [Fact]
        public void DuplicateId_IsRejected()
        {
            var engine = NewEngine();
            var resp = engine.Update(Reorder(10, 10, "P3", "P3"));
            Assert.Contains(resp.Alerts, a => a.Code == "INVALID_STOP_ORDER");
            Assert.Equal(new[] { "B", "P1", "P2", "P3", "B" }, Ids(resp));
        }

        [Fact]
        public void ResendingTheSameOrder_DoesNotReplanAgain()
        {
            var engine = NewEngine();
            engine.Update(Reorder(10, 10, "P3", "P2", "P1"));
            var again = engine.Update(Reorder(10, 10, "P3", "P2", "P1"));
            Assert.False(again.Route.Replanned);
            Assert.Equal("P3", again.Route.Target);
        }

        [Fact]
        public void ReorderedSequence_DrivesProgress()
        {
            var engine = NewEngine();
            engine.Update(Reorder(10, 10, "P3", "P2", "P1"));
            var atP3 = engine.Update(At(10, 100));
            Assert.Equal("done", atP3.Objectives.Single(o => o.Id == "P3").Status);
            Assert.Equal("P2", atP3.NextWaypoint.Id);
            Assert.Equal("pending", atP3.Objectives.Single(o => o.Id == "P1").Status);
        }

        [Fact]
        public void Reorder_OnlyAffectsItsOwnSession()
        {
            var engine = NewEngine();
            engine.Update(Reorder(10, 10, "P3", "P2", "P1"));
            var other = engine.Update(At(10, 10, "s2"));
            Assert.Equal(new[] { "B", "P1", "P2", "P3", "B" }, Ids(other));
            Assert.Equal("P1", other.NextWaypoint.Id);
        }

        [Fact]
        public void ReorderWhenOnlyTheReturnIsLeft_IsAcceptedAndChangesNothing()
        {
            var engine = NewEngine();
            engine.Update(At(10, 10));
            engine.Update(At(100, 10));
            engine.Update(At(100, 100));
            engine.Update(At(10, 100));                                  // all stops done, heading home
            var resp = engine.Update(Reorder(10, 100, "P2", "P1"));
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "INVALID_STOP_ORDER");
            Assert.Equal("B", resp.NextWaypoint.Id);
        }

        [Fact]
        public void BackgroundMode_RouteFollowsTheNewOrder_NotAStaleSearch()
        {
            using var engine = NewEngine(new NavigationOptions { BackgroundReplanning = true });
            engine.Update(At(10, 10));                                   // queues a search to P1
            NavigationResponse latest = engine.Update(Reorder(10, 10, "P3", "P2", "P1"));

            var clock = Stopwatch.StartNew();
            while ((latest.Route.Pending || latest.Route.Target != "P3" || !latest.Route.Found) && clock.ElapsedMilliseconds < 5000)
            {
                Thread.Sleep(5);
                latest = engine.Update(At(10, 10));
                // Whatever is shown must never be the old P1 route labelled as the new target.
                if (latest.Route.Found) Assert.Equal("P3", latest.Route.Target);
            }

            Assert.Equal("P3", latest.Route.Target);
            Assert.True(latest.Route.Found);
            var end = latest.Route.Points[latest.Route.Points.Count - 1];
            Assert.True(end.DistanceTo(new Point2(10, 100)) < 3.0);
        }

        [Fact]
        public void PlanItinerary_ChainsLegsThroughTheGivenOrder()
        {
            var engine = NewEngine();
            List<ItineraryLeg> legs = engine.PlanItinerary(new Point2(10, 10), new[] { "P3", "P2", "P1", "B" });

            Assert.Equal(new[] { "P3", "P2", "P1", "B" }, legs.Select(l => l.TargetId).ToArray());
            Assert.All(legs, l => Assert.True(l.Route.Found));
            Assert.Equal(new Point2(10, 10), legs[0].From);
            Assert.Equal(new Point2(10, 100), legs[1].From);             // each leg starts where the previous ended
            Assert.Equal(new Point2(100, 10), legs[3].From);
            double total = legs.Sum(l => l.Route.HorizontalLengthM);
            Assert.InRange(total, 355, 400);                             // the 4 sides of the 90 m square
        }

        [Fact]
        public void PlanItinerary_UnknownPlace_Throws()
        {
            var engine = NewEngine();
            Assert.Throws<ArgumentException>(() => engine.PlanItinerary(new Point2(10, 10), new[] { "P1", "NOPE" }));
            Assert.Throws<ArgumentNullException>(() => engine.PlanItinerary(new Point2(10, 10), null));
        }
    }
}
