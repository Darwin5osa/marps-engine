using System;
using System.Linq;
using Marps.Engine.Abstractions;
using Marps.Engine.Data;
using Marps.Engine.Geometry;
using Marps.Engine.Mission;
using Marps.Engine.Navigation;
using Marps.Engine.Tests.Helpers;
using Xunit;
using static Marps.Engine.Tests.Interface.Fixtures;

namespace Marps.Engine.Tests.Interface
{
    /// <summary>Regression tests for anomalies found by code review (each one failed before its fix).</summary>
    [Trait("Method", "Update")]
    public class AnomalyRegressionTests
    {
        [Theory]
        [InlineData(double.NaN, 10)]
        [InlineData(10, double.NaN)]
        [InlineData(double.PositiveInfinity, 10)]
        [InlineData(10, double.NegativeInfinity)]
        public void NonFiniteSample_DoesNotCreditTheActiveStop(double x, double z)
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));                       // B done, P1 active
            var bad = engine.Update(At(x, z));
            Assert.Equal("active", bad.Objectives[1].Status);
            var back = engine.Update(At(BasePos));            // a valid sample right after the bad one
            Assert.Equal("active", back.Objectives[1].Status);
            Assert.Equal("P1", back.Route.Target);
        }

        [Fact]
        public void NonFiniteSample_RaisesInvalidPosition_AndHasNoRouteOrNextWaypoint()
        {
            var resp = NewEngine().Update(At(double.NaN, 10));
            Assert.Contains(resp.Alerts, a => a.Code == "INVALID_POSITION" && a.Severity == "danger");
            Assert.Null(resp.Route);
            Assert.Null(resp.NextWaypoint);
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "NO_ROUTE");
        }

        [Theory]
        [InlineData(double.NaN, 10, 0)]
        [InlineData(10, double.NaN, 0)]
        [InlineData(10, 10, double.NaN)]
        [InlineData(10, 10, double.PositiveInfinity)]
        public void ResponseNumbers_AreAlwaysFinite(double x, double z, double heading)
        {
            var resp = NewEngine().Update(At(x, z, heading: heading));
            foreach (double v in Numbers(resp))
                Assert.True(double.IsFinite(v), $"non-finite value {v} in the response");
        }

        [Fact]
        public void NonFiniteHeading_RaisesInvalidHeading_AndKeepsNavigating()
        {
            var resp = NewEngine().Update(At(BasePos.X, BasePos.Z, heading: double.NaN));
            Assert.Contains(resp.Alerts, a => a.Code == "INVALID_HEADING");
            Assert.True(resp.Route.Found);
            Assert.Equal(0.0, resp.NextWaypoint.RelativeBearingDeg);
        }

        [Fact]
        public void SteepTerrainAhead_IsDanger_EvenWithRoughTerrainBeforeIt()
        {
            // Heading north from (60, 20): rough (3) at z = 24, blocked slope (2) at z = 30. Code 3 > 2, so a plain
            // max() used to report the softer "rough" warning instead of the danger.
            var map = TestTerrain.WithHazard(Flat(), (c, r) =>
            {
                double x = c * Cell, z = (Nodes - 1 - r) * Cell;
                if (Math.Abs(x - 60) > 1) return 0;
                if (Math.Abs(z - 24) < 1) return 3;
                if (Math.Abs(z - 30) < 1) return 2;
                return 0;
            });
            var resp = NewEngine(map: map).Update(At(60, 20, heading: 0));
            var alert = Assert.Single(resp.Alerts, a => a.Code == "HAZARD_AHEAD");
            Assert.Equal("danger", alert.Severity);
        }

        [Fact]
        public void RoughTerrainAhead_MessageIsInEnglish()
        {
            var map = TestTerrain.WithHazard(Flat(), (c, r) => (byte)(Math.Abs(c * Cell - 60) < 1 && Math.Abs((Nodes - 1 - r) * Cell - 24) < 1 ? 3 : 0));
            var resp = NewEngine(map: map).Update(At(60, 20, heading: 0));
            var alert = Assert.Single(resp.Alerts, a => a.Code == "HAZARD_AHEAD");
            Assert.Equal("warning", alert.Severity);
            Assert.Equal("Rough terrain ahead.", alert.Message);
        }

        [Fact]
        public void UnreachableTarget_IsNotSearchedAgainEveryFrame()
        {
            var engine = NewEngine(map: Walled(P1Pos));
            Assert.True(engine.Update(At(BasePos)).Route.Replanned);
            var still = engine.Update(At(BasePos.X + 1, BasePos.Z));          // barely moved
            Assert.False(still.Route.Replanned);
            Assert.Contains(still.Alerts, a => a.Code == "NO_ROUTE");
            Assert.True(engine.Update(At(BasePos.X + 20, BasePos.Z)).Route.Replanned);   // moved > OffRouteM
        }

        [Fact]
        public void UnreachableTarget_ForceReplan_SearchesAgain()
        {
            var engine = NewEngine(map: Walled(P1Pos));
            engine.Update(At(BasePos));
            var req = At(BasePos);
            req.ForceReplan = true;
            Assert.True(engine.Update(req).Route.Replanned);
        }

        [Fact]
        public void UnknownActiveWaypoint_RaisesAnAlert()
        {
            var req = At(BasePos);
            req.ActiveWaypointId = "NOPE";
            var resp = NewEngine().Update(req);
            Assert.Contains(resp.Alerts, a => a.Code == "INVALID_WAYPOINT" && a.Severity == "warning");
            Assert.Equal("P1", resp.Route.Target);
        }

        [Fact]
        public void ActiveWaypointAlreadyDone_IsIgnoredSilently()
        {
            // Unity may keep sending "B" every frame after the return; that must not spam alerts.
            var engine = NewEngine();
            CompleteMission(engine);
            var req = At(BasePos);
            req.ActiveWaypointId = "B";
            var resp = engine.Update(req);
            Assert.True(resp.MissionComplete);
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "INVALID_WAYPOINT");
        }

        [Theory]
        [InlineData(double.NaN, 10, 0)]
        [InlineData(10, 10, double.NaN)]
        public void BrokenSample_ResponseStillSerializesToJson(double x, double z, double heading)
        {
            var req = At(x, z, heading: heading);
            req.MissionTimeS = double.NaN;
            var resp = NewEngine().Update(req);
            string json = MarpsJson.Serialize(resp);                 // NaN would make System.Text.Json throw
            Assert.Contains("\"sessionId\":\"s1\"", json);
            Assert.Equal(0.0, resp.MissionTimeS);
        }

        [Fact]
        public void BrokenSample_StillAppliesTheEmergencyButton()
        {
            var engine = NewEngine();
            ReachP1(engine);
            var req = At(double.NaN, double.NaN);
            req.EmergencyReturn = true;
            var bad = engine.Update(req);
            Assert.True(bad.Emergency.Active);
            Assert.False(bad.Emergency.Arrived);
            var next = engine.Update(At(P1Pos));
            Assert.Equal("B", next.Route.Target);
            Assert.True(next.Route.Found);
        }

        [Fact]
        public void BrokenSample_EmergencyInBackgroundMode_RoutesHomeOnTheFirstValidSample()
        {
            using var engine = NewEngine(options: new NavigationOptions { BackgroundReplanning = true });
            Settle(engine, At(BasePos));
            var req = At(double.NaN, 10);
            req.EmergencyReturn = true;
            engine.Update(req);
            var next = engine.Update(At(P1Pos));
            Assert.False(next.Route.Pending);                         // computed at once, not queued
            Assert.Equal("B", next.Route.Target);
            Assert.True(next.Route.Found);
        }

        [Fact]
        public void BrokenSample_DoesNotMoveTheSession()
        {
            // The previous valid position is kept, so the path check after the bad sample still works.
            var engine = NewEngine();
            engine.Update(At(BasePos));
            engine.Update(At(50, 10));
            engine.Update(At(double.NaN, double.NaN));
            var resp = engine.Update(At(P1Pos.X + 30, P1Pos.Z));      // the segment 50 -> 130 passes over P1
            Assert.Equal("done", resp.Objectives[1].Status);
        }

        [Fact]
        public void UnreachableTarget_InBackgroundMode_IsNotQueuedAgainEveryFrame()
        {
            using var engine = NewEngine(map: Walled(P1Pos), options: new NavigationOptions { BackgroundReplanning = true });
            var first = Settle(engine, At(BasePos));
            Assert.False(first.Route.Found);
            for (int i = 0; i < 5; i++)
            {
                var again = engine.Update(At(BasePos));
                Assert.False(again.Route.Pending);
                Assert.Contains(again.Alerts, a => a.Code == "NO_ROUTE");
            }
        }

        [Fact]
        public void Service_EngineException_IsReported_AndTheLoopGoesOn()
        {
            var service = new NavigationService(new ThrowingEngine(), new FixedSensor(), null);
            var resp = service.Step();
            Assert.Equal(1, service.EngineFaults);
            Assert.Equal("astro", resp.SessionId);
            Assert.Contains(resp.Alerts, a => a.Code == "ENGINE_FAULT" && a.Severity == "danger");
            service.Step();
            Assert.Equal(2, service.EngineFaults);
        }

        private sealed class FixedSensor : IAstronautSensor
        {
            public NavigationRequest Read() => At(10, 10, "astro");
        }

        private sealed class ThrowingEngine : INavigationEngine
        {
            public Marps.Engine.Terrain.TerrainMap Map => null;
            public MissionDefinition Mission => null;
            public Waypoint Base => null;
            public NavigationResponse Update(NavigationRequest request) => throw new InvalidOperationException("boom");
            public NavigationResponse ReorderStops(NavigationRequest current, System.Collections.Generic.IList<string> stopOrder) => Update(current);
            public NavigationResponse StartEmergencyReturn(NavigationRequest current) => Update(current);
            public NavigationResponse CancelEmergencyReturn(NavigationRequest current) => Update(current);
            public Marps.Engine.Routing.RouteResult PlanRoute(Point2 from, Point2 to) => null;
            public System.Collections.Generic.List<ItineraryLeg> PlanItinerary(Point2 from, System.Collections.Generic.IEnumerable<string> placeIds) => null;
            public TripPlan PlanTrip(TripRequest request) => null;
            public TripPlan PlanTrip(Point2 current, System.Collections.Generic.IEnumerable<Point2> stops, Point2 destination) => null;
            public Marps.Engine.Routing.CostField GetCostField(string placeId) => null;
            public Marps.Engine.Routing.CostField GetCostField(Point2 target) => null;
            public double SpeedAt(double slopeDeg, bool uphill) => 0;
            public void Dispose() { }
        }

        /// <summary>Every double in the response graph (properties, lists and nested objects), found by reflection.</summary>
        internal static System.Collections.Generic.List<double> Numbers(object root)
        {
            var found = new System.Collections.Generic.List<double>();
            void Walk(object o)
            {
                if (o == null) return;
                if (o is double d) { found.Add(d); return; }
                if (o is Point2 p) { found.Add(p.X); found.Add(p.Z); return; }
                var t = o.GetType();
                if (t.IsPrimitive || o is string) return;
                if (o is System.Collections.IEnumerable items) { foreach (var item in items) Walk(item); return; }
                foreach (var prop in t.GetProperties().Where(pr => pr.GetIndexParameters().Length == 0))
                    Walk(prop.GetValue(o));
            }
            Walk(root);
            return found;
        }
    }
}
