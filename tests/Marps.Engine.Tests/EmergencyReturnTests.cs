using System.Diagnostics;
using System.Linq;
using System.Threading;
using Marps.Engine.Geometry;
using Marps.Engine.Json;
using Marps.Engine.Mission;
using Marps.Engine.Navigation;
using Marps.Engine.Tests.Helpers;
using Xunit;
using static Marps.Engine.Tests.StopOrderTests;

namespace Marps.Engine.Tests
{
    public class EmergencyReturnTests
    {
        private static NavigationEngine NewEngine(NavigationOptions options = null) =>
            new NavigationEngine(TestTerrain.Flat(60, 2.0), SquareMission(), options);

        private static NavigationRequest Emergency(double x, double z, string session = "s1")
        {
            var req = At(x, z, session);
            req.EmergencyReturn = true;
            req.MissionTimeS = 1234;
            return req;
        }

        /// <summary>Leaves the base and completes P1; the astronaut is then heading to P2.</summary>
        private static void WalkToP1(NavigationEngine engine)
        {
            engine.Update(At(10, 10));
            engine.Update(At(100, 10));
        }

        [Fact]
        public void Emergency_RoutesStraightToBase_SkippingPendingStops()
        {
            var engine = NewEngine();
            WalkToP1(engine);

            var resp = engine.Update(Emergency(100, 50));

            Assert.Equal("B", resp.NextWaypoint.Id);
            Assert.Equal("B", resp.Route.Target);
            Assert.True(resp.Route.Found);
            Assert.True(resp.Route.Replanned);
            var end = resp.Route.Points[resp.Route.Points.Count - 1];
            Assert.True(end.DistanceTo(new Point2(10, 10)) < 3.0);

            Assert.NotNull(resp.Emergency);
            Assert.True(resp.Emergency.Active);
            Assert.False(resp.Emergency.Arrived);
            Assert.Equal("B", resp.Emergency.BaseId);
            Assert.Equal(1234, resp.Emergency.ActivatedAtS);

            Assert.Equal("EMERGENCY_RETURN", resp.Alerts[0].Code);      // the most important alert comes first
            Assert.Equal("danger", resp.Alerts[0].Severity);
            Assert.False(resp.MissionComplete);

            // Pending stops stay pending (they resume on cancel); the return step is the active one.
            Assert.Equal(new[] { "done", "done", "pending", "pending", "active" }, resp.Objectives.Select(o => o.Status).ToArray());
        }

        [Fact]
        public void Emergency_StaysOnWithoutResendingTheFlag()
        {
            var engine = NewEngine();
            WalkToP1(engine);
            engine.Update(Emergency(100, 50));

            var next = engine.Update(At(95, 45));
            Assert.True(next.Emergency.Active);
            Assert.Equal("B", next.NextWaypoint.Id);
            Assert.Equal(1234, next.Emergency.ActivatedAtS);             // pressing again would not reset it either
        }

        [Fact]
        public void Emergency_PassingByAStop_DoesNotCompleteIt()
        {
            var engine = NewEngine();
            WalkToP1(engine);
            engine.Update(Emergency(100, 50));

            var resp = engine.Update(At(100, 100));                     // right on P2
            Assert.Equal("pending", resp.Objectives.Single(o => o.Id == "P2").Status);
            Assert.Equal("B", resp.NextWaypoint.Id);
        }

        [Fact]
        public void Emergency_IgnoresActiveWaypointRequests()
        {
            var engine = NewEngine();
            WalkToP1(engine);
            engine.Update(Emergency(100, 50));
            var req = At(100, 50);
            req.ActiveWaypointId = "P3";
            Assert.Equal("B", engine.Update(req).NextWaypoint.Id);
        }

        [Fact]
        public void Emergency_ReportsArrivalAtBase()
        {
            var engine = NewEngine();
            WalkToP1(engine);
            engine.Update(Emergency(100, 50));

            var home = engine.Update(At(12, 12));
            Assert.True(home.Emergency.Arrived);
            Assert.Contains(home.Alerts, a => a.Code == "EMERGENCY_RETURN" && a.Severity == "info");
            Assert.DoesNotContain(home.Alerts, a => a.Code == "EMERGENCY_RETURN" && a.Severity == "danger");
        }

        [Fact]
        public void Cancel_ResumesTheMissionAtTheStopThatWasActive()
        {
            var engine = NewEngine();
            WalkToP1(engine);
            engine.Update(Emergency(100, 50));

            var req = At(60, 30);
            req.CancelEmergency = true;
            var resp = engine.Update(req);

            Assert.Null(resp.Emergency);
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "EMERGENCY_RETURN");
            Assert.Equal("P2", resp.NextWaypoint.Id);
            Assert.Equal("P2", resp.Route.Target);
            Assert.True(resp.Route.Replanned);
            Assert.Equal(new[] { "done", "done", "active", "pending", "pending" }, resp.Objectives.Select(o => o.Status).ToArray());
        }

        [Fact]
        public void Emergency_KeepsAReorderedSequenceForWhenItIsCancelled()
        {
            var engine = NewEngine();
            WalkToP1(engine);
            engine.Update(Emergency(100, 50));
            var reorder = At(100, 50);
            reorder.StopOrder = new System.Collections.Generic.List<string> { "P3", "P2" };
            Assert.Equal("B", engine.Update(reorder).NextWaypoint.Id);   // still going home

            var cancel = At(100, 50);
            cancel.CancelEmergency = true;
            Assert.Equal("P3", engine.Update(cancel).NextWaypoint.Id);
        }

        [Fact]
        public void Emergency_OnlyAffectsItsOwnSession()
        {
            var engine = NewEngine();
            engine.Update(Emergency(50, 50, "s1"));
            var other = engine.Update(At(10, 10, "s2"));                // at the base: departure step done
            Assert.Null(other.Emergency);
            Assert.Equal("P1", other.NextWaypoint.Id);
        }

        [Fact]
        public void MissionWithoutBase_ReportsNoBase()
        {
            var mission = new MissionDefinition("nobase", "test",
                new[] { new Waypoint { Id = "P1", Type = "science", Position = new Point2(60, 60) } },
                new[] { "P1" }, null);
            var engine = new NavigationEngine(TestTerrain.Flat(60, 2.0), mission);

            var resp = engine.Update(Emergency(10, 10));
            Assert.Null(engine.Base);
            Assert.Null(resp.Emergency);
            Assert.Contains(resp.Alerts, a => a.Code == "NO_BASE" && a.Severity == "danger");
            Assert.Equal("P1", resp.NextWaypoint.Id);
        }

        [Fact]
        public void Base_IsFoundEvenWhenItIsNotPartOfTheSequence()
        {
            var mission = new MissionDefinition("m", "test",
                new[]
                {
                    new Waypoint { Id = "P1", Type = "science", Position = new Point2(60, 60) },
                    new Waypoint { Id = "HAB", Type = "base", Position = new Point2(20, 20) },
                },
                new[] { "P1" }, null);
            var engine = new NavigationEngine(TestTerrain.Flat(60, 2.0), mission);

            var resp = engine.Update(Emergency(100, 100));
            Assert.Equal("HAB", engine.Base.Id);
            Assert.Equal("HAB", resp.NextWaypoint.Id);
            Assert.True(resp.Route.Found);
            Assert.Equal(new[] { "pending" }, resp.Objectives.Select(o => o.Status).ToArray()); // no return step to mark
        }

        [Fact]
        public void BackgroundMode_EmergencyRouteIsImmediate_AndNotReplacedByAStaleSearch()
        {
            using var engine = NewEngine(new NavigationOptions { BackgroundReplanning = true });
            var first = engine.Update(At(100, 50));
            Assert.True(first.Route.Pending);                            // search to P1 queued in the background

            var sos = engine.Update(Emergency(100, 50));
            Assert.True(sos.Route.Found);                                // computed on this very call
            Assert.Equal("B", sos.Route.Target);

            var clock = Stopwatch.StartNew();
            while (clock.ElapsedMilliseconds < 300)
            {
                Thread.Sleep(5);
                var r = engine.Update(At(100, 50));
                Assert.Equal("B", r.Route.Target);
                Assert.True(r.Route.Found);
                var end = r.Route.Points[r.Route.Points.Count - 1];
                Assert.True(end.DistanceTo(new Point2(10, 10)) < 3.0);
            }
        }

        [Fact]
        public void Json_CommandsAndEmergencyBlockUseTheContractNames()
        {
            var req = MarpsJson.Deserialize<NavigationRequest>(
                "{\"sessionId\":\"j\",\"x\":100,\"z\":50,\"headingDeg\":0,\"emergencyReturn\":true,\"stopOrder\":[\"P3\"]}");
            Assert.True(req.EmergencyReturn);
            Assert.Equal(new[] { "P3" }, req.StopOrder);

            var engine = NewEngine();
            string json = MarpsJson.Serialize(engine.Update(req));
            Assert.Contains("\"emergency\":{\"active\":true,\"baseId\":\"B\"", json);

            string calm = MarpsJson.Serialize(engine.Update(At(100, 50, "other")));
            Assert.DoesNotContain("\"emergency\"", calm);               // omitted when not active
        }
    }
}
