using System;
using System.Collections.Generic;
using System.Linq;
using Marps.Engine.Abstractions;
using Marps.Engine.Geometry;
using Marps.Engine.Json;
using Marps.Engine.Mission;
using Marps.Engine.Navigation;
using Marps.Engine.Routing;
using Marps.Engine.Terrain;
using Marps.Engine.Tests.Helpers;
using Xunit;

namespace Marps.Engine.Tests
{
    public class NavigationTests
    {
        // Synthetic 2 km x 2 km map with 2 m cells, all flat except what each test adds.
        private static MissionDefinition SimpleMission(TerrainMap map, params (string Id, string Type, double X, double Z)[] places)
        {
            var wps = places.Select(p => new Waypoint
            {
                Id = p.Id,
                Type = p.Type,
                Name = p.Id,
                Position = new Point2(p.X, p.Z),
            }).ToList();
            var objectives = places.Where(p => p.Type == "science" || p.Type == "base").Select(p => p.Id).ToList();
            return new MissionDefinition("prueba", "obj", wps, objectives, new List<HazardZone>());
        }

        private static NavigationRequest At(double x, double z, double heading = 0, string session = "s1") =>
            new NavigationRequest { SessionId = session, X = x, Z = z, HeadingDeg = heading, PitchDeg = 0 };

        [Theory]
        [InlineData(0, 0)]    // looking north: target to the north -> 0
        [InlineData(90, -90)] // looking east: target ends up on the left -> -90
        [InlineData(180, 180)]
        [InlineData(270, 90)] // looking west: target ends up on the right -> +90
        public void NextWaypoint_RelativeBearingFollowsLookDirection(double heading, double expectedRelative)
        {
            var map = TestTerrain.Flat(60, 2.0);
            var mission = SimpleMission(map, ("B", "base", 10, 10), ("P1", "science", 10, 100));
            var engine = new NavigationEngine(map, mission);

            var resp = engine.Update(At(10, 10, heading));
            Assert.Equal("P1", resp.NextWaypoint.Id);
            Assert.Equal(0.0, resp.NextWaypoint.BearingDeg, 6);
            Assert.Equal(expectedRelative, resp.NextWaypoint.RelativeBearingDeg, 6);
            Assert.Equal(90.0, resp.NextWaypoint.DistanceM, 6);
        }

        [Fact]
        public void Objective_ReachedWithinRadius_MarksDoneAndAdvances()
        {
            var map = TestTerrain.Flat(60, 2.0);
            var mission = SimpleMission(map, ("B", "base", 10, 10), ("P1", "science", 60, 10), ("P2", "science", 60, 60));
            var engine = new NavigationEngine(map, mission);

            engine.Update(At(10, 10));                       // inicio
            var resp = engine.Update(At(58, 10));           // 2 m from P1: reached
            Assert.Equal("done", resp.Objectives.First(o => o.Id == "P1").Status);
            Assert.Equal("P2", resp.NextWaypoint.Id);
            Assert.Equal("active", resp.Objectives.First(o => o.Id == "P2").Status);
        }

        [Fact]
        public void Objective_NotReachedOutsideRadius()
        {
            var map = TestTerrain.Flat(60, 2.0);
            var mission = SimpleMission(map, ("P1", "science", 60, 10));
            var engine = new NavigationEngine(map, mission);
            var resp = engine.Update(At(40, 10));
            Assert.Equal("active", resp.Objectives[0].Status);
        }

        [Fact]
        public void Objective_ReachedWhenPathSweepsPastIt_EvenIfNoSampleWasInside()
        {
            // Slow sampling: the astronaut jumps from x=0 to x=100. P1 (50,10) lies on the leg; P2 (90,50) does not.
            var map = TestTerrain.Flat(60, 2.0);
            var mission = SimpleMission(map, ("P1", "science", 50, 10), ("P2", "science", 90, 50));
            var engine = new NavigationEngine(map, mission);

            engine.Update(At(0, 10));
            var resp = engine.Update(At(100, 10));
            Assert.Equal("done", resp.Objectives.First(o => o.Id == "P1").Status);
            Assert.Equal("P2", resp.NextWaypoint.Id);
        }

        [Fact]
        public void ActiveWaypoint_ChosenByAstronaut_IsHonoured()
        {
            var map = TestTerrain.Flat(60, 2.0);
            var mission = SimpleMission(map, ("P1", "science", 20, 20), ("P2", "science", 80, 80));
            var engine = new NavigationEngine(map, mission);
            var req = At(10, 10);
            req.ActiveWaypointId = "P2";
            var resp = engine.Update(req);
            Assert.Equal("P2", resp.NextWaypoint.Id);
        }

        [Fact]
        public void ActiveWaypoint_AlreadyDone_FallsBackToNext()
        {
            var map = TestTerrain.Flat(60, 2.0);
            var mission = SimpleMission(map, ("P1", "science", 20, 20), ("P2", "science", 80, 80));
            var engine = new NavigationEngine(map, mission);
            engine.Update(At(20, 20)); // P1 cumplido
            var req = At(20, 20);
            req.ActiveWaypointId = "P1";
            Assert.Equal("P2", engine.Update(req).NextWaypoint.Id);
        }

        [Fact]
        public void Mission_RequiresReturningToBase_BeforeCompleting()
        {
            // Sequence B -> P1 -> B: when P1 is completed, the active target must be returning to base.
            var map = TestTerrain.Flat(60, 2.0);
            var mission = new MissionDefinition("m", "o",
                new[]
                {
                    new Waypoint { Id = "B", Type = "base", Position = new Point2(10, 10) },
                    new Waypoint { Id = "P1", Type = "science", Position = new Point2(60, 10) },
                },
                new[] { "B", "P1", "B" }, new HazardZone[0]);
            var engine = new NavigationEngine(map, mission);

            engine.Update(At(10, 10));                 // leaves the base: step B completed
            var atP1 = engine.Update(At(60, 10));      // llega a P1
            Assert.Equal("B", atP1.NextWaypoint.Id);   // falta volver
            Assert.False(atP1.MissionComplete);
            Assert.Equal(new[] { "done", "done", "active" }, atP1.Objectives.Select(o => o.Status).ToArray());

            var home = engine.Update(At(10, 10));      // regresa
            Assert.True(home.MissionComplete);
            Assert.Equal(new[] { "done", "done", "done" }, home.Objectives.Select(o => o.Status).ToArray());
        }

        [Fact]
        public void MissionComplete_WhenAllObjectivesReached()
        {
            var map = TestTerrain.Flat(60, 2.0);
            var mission = SimpleMission(map, ("P1", "science", 20, 20));
            var engine = new NavigationEngine(map, mission);
            var resp = engine.Update(At(20, 20));
            Assert.True(resp.MissionComplete);
            Assert.Null(resp.NextWaypoint);
            Assert.Null(resp.Route);
        }

        [Fact]
        public void Route_PlannedOnceThenCached_ReplansWhenOffRoute()
        {
            var map = TestTerrain.Flat(80, 2.0);
            var mission = SimpleMission(map, ("P1", "science", 150, 10));
            var engine = new NavigationEngine(map, mission, new NavigationOptions { OffRouteM = 15 });

            var first = engine.Update(At(10, 10));
            Assert.True(first.Route.Replanned);
            Assert.True(first.Route.Found);

            var second = engine.Update(At(20, 10)); // on the route
            Assert.False(second.Route.Replanned);

            var off = engine.Update(At(60, 60));    // far from the route
            Assert.True(off.Route.Replanned);
        }

        [Fact]
        public void Route_ForceReplan_RecomputesOnDemand()
        {
            var map = TestTerrain.Flat(80, 2.0);
            var mission = SimpleMission(map, ("P1", "science", 150, 10));
            var engine = new NavigationEngine(map, mission);
            engine.Update(At(10, 10));
            var req = At(10, 10);
            req.ForceReplan = true;
            Assert.True(engine.Update(req).Route.Replanned);
        }

        [Fact]
        public void Route_RemainingDistanceShrinksAsAstronautWalks()
        {
            var map = TestTerrain.Flat(80, 2.0);
            var mission = SimpleMission(map, ("P1", "science", 150, 10));
            var engine = new NavigationEngine(map, mission);
            var a = engine.Update(At(10, 10)).Route.RemainingM;
            var b = engine.Update(At(60, 10)).Route.RemainingM;
            Assert.True(b < a);
            Assert.Equal(90, b, 0); // quedan 90 m hasta x=150
        }

        [Fact]
        public void Route_PolylineStartsAtAstronaut()
        {
            var map = TestTerrain.Flat(80, 2.0);
            var mission = SimpleMission(map, ("P1", "science", 150, 10));
            var engine = new NavigationEngine(map, mission);
            var resp = engine.Update(At(10.3, 10.1));
            Assert.Equal(10.3, resp.Route.Points[0].X, 6);
            Assert.Equal(10.1, resp.Route.Points[0].Z, 6);
        }

        [Fact]
        public void NoRoute_RaisesNoRouteAlert()
        {
            // P1 surrounded by a 100 m wall: no route.
            var baseMap = TestTerrain.Build(80, 2.0, (x, z) => (x > 100 && x < 110) ? 100.0 : 0.0);
            var mission = SimpleMission(baseMap, ("P1", "science", 140, 10));
            var engine = new NavigationEngine(baseMap, mission);
            var resp = engine.Update(At(10, 10));
            Assert.False(resp.Route.Found);
            Assert.Contains(resp.Alerts, a => a.Code == "NO_ROUTE" && a.Severity == "danger");
        }

        [Fact]
        public void HazardAhead_DangerWhenLookingAtSteepTerrain()
        {
            // 30° wall (k = tan 30°) starting at x = 20 m, visible when looking east.
            double k = Math.Tan(30 * Math.PI / 180);
            var map = TestTerrain.Build(60, 2.0, (x, z) => x >= 20 ? k * (x - 20) : 0);
            var mission = SimpleMission(map, ("P1", "science", 5, 5));
            var engine = new NavigationEngine(map, mission);

            var east = engine.Update(At(5, 5, heading: 90));
            Assert.Contains(east.Alerts, a => a.Code == "HAZARD_AHEAD" && a.Severity == "danger");

            var west = engine.Update(At(5, 5, heading: 270));
            Assert.DoesNotContain(west.Alerts, a => a.Code == "HAZARD_AHEAD");
        }

        [Fact]
        public void HazardAhead_WarningOnModerateSlope()
        {
            double k = Math.Tan(18 * Math.PI / 180);
            var map = TestTerrain.Build(60, 2.0, (x, z) => x >= 20 ? k * (x - 20) : 0);
            var mission = SimpleMission(map, ("P1", "science", 5, 5));
            var resp = new NavigationEngine(map, mission).Update(At(5, 5, heading: 90));
            var alert = resp.Alerts.FirstOrDefault(a => a.Code == "HAZARD_AHEAD");
            Assert.NotNull(alert);
            Assert.Equal("warning", alert.Severity);
        }

        [Fact]
        public void SignalLoss_WhenRidgeBlocksAllAntennas()
        {
            // 300 m ridge between the astronaut (x=10) and the masts (x >= 150).
            var map = TestTerrain.Build(120, 2.0, (x, z) => (x > 60 && x < 66) ? 300.0 : 0.0);
            var mission = SimpleMission(map,
                ("P1", "science", 200, 100),
                ("A2", "antenna", 180, 100),
                ("A3", "antenna", 200, 40));
            var resp = new NavigationEngine(map, mission).Update(At(10, 100));
            Assert.Equal(0, resp.Layers.VisibleAntennas);
            Assert.Contains(resp.Alerts, a => a.Code == "SIGNAL_LOSS");
        }

        [Fact]
        public void SignalClear_OnOpenTerrain()
        {
            var map = TestTerrain.Flat(60, 2.0);
            var mission = SimpleMission(map,
                ("P1", "science", 100, 100),
                ("A2", "antenna", 80, 80),
                ("A3", "antenna", 40, 90));
            var resp = new NavigationEngine(map, mission).Update(At(10, 10));
            Assert.Equal(2, resp.Layers.VisibleAntennas);
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "SIGNAL_LOSS");
        }

        [Fact]
        public void LineOfSight_BlockedByTallWall_ClearWithoutWall()
        {
            var flat = TestTerrain.Flat(60, 2.0);
            Assert.True(LineOfSight.Clear(flat, new Point2(10, 10), 1.7, new Point2(100, 10), 3.0));

            var wall = TestTerrain.Build(60, 2.0, (x, z) => (x > 50 && x < 55) ? 40.0 : 0.0);
            Assert.False(LineOfSight.Clear(wall, new Point2(10, 10), 1.7, new Point2(100, 10), 3.0));
        }

        [Fact]
        public void SolarStorm_AlertWhenWeatherReportsStorm()
        {
            var map = TestTerrain.Flat(40, 2.0);
            var mission = SimpleMission(map, ("P1", "science", 60, 60));
            var engine = new NavigationEngine(map, mission);
            var req = At(10, 10);
            req.SpaceWeather = new SpaceWeather { StormActive = true, Level = "G3", Source = "test" };
            var resp = engine.Update(req);
            Assert.Contains(resp.Alerts, a => a.Code == "SOLAR_STORM" && a.Message.Contains("G3"));
        }

        [Fact]
        public void SolarStorm_NoAlertWhenQuiet()
        {
            var map = TestTerrain.Flat(40, 2.0);
            var mission = SimpleMission(map, ("P1", "science", 60, 60));
            var req = At(10, 10);
            req.SpaceWeather = new SpaceWeather { StormActive = false };
            Assert.DoesNotContain(new NavigationEngine(map, mission).Update(req).Alerts, a => a.Code == "SOLAR_STORM");
        }

        [Fact]
        public void OffMap_IsReported()
        {
            var map = TestTerrain.Flat(40, 2.0);
            var mission = SimpleMission(map, ("P1", "science", 60, 60));
            var resp = new NavigationEngine(map, mission).Update(At(-50, 10));
            Assert.False(resp.Position.Inside);
            Assert.Contains(resp.Alerts, a => a.Code == "OFF_MAP");
        }

        [Fact]
        public void Hazards_ListsZonesWithinRadiusSortedByDistance()
        {
            var map = TestTerrain.Flat(60, 2.0);
            var wps = new List<Waypoint> { new Waypoint { Id = "P1", Position = new Point2(10, 10), IsObjective = true } };
            var zones = new List<HazardZone>
            {
                new HazardZone { Type = "far", BoxMin = new Point2(1000, 1000), BoxMax = new Point2(1010, 1010), MaxSlopeDeg = 30 },
                new HazardZone { Type = "near", BoxMin = new Point2(40, 10), BoxMax = new Point2(50, 20), MaxSlopeDeg = 44 },
            };
            var mission = new MissionDefinition("m", "o", wps, new[] { "P1" }, zones);
            var resp = new NavigationEngine(map, mission).Update(At(10, 10));

            Assert.Single(resp.Hazards);
            Assert.Equal("near", resp.Hazards[0].Type);
            Assert.Equal("high", resp.Hazards[0].Severity);
            Assert.Equal(30.0, resp.Hazards[0].DistanceM, 6);
        }

        [Fact]
        public void Layers_ReportSlopeAndHazardClassAtPosition()
        {
            double k = Math.Tan(10 * Math.PI / 180);
            var map = TestTerrain.Build(60, 2.0, (x, z) => k * x);
            var mission = SimpleMission(map, ("P1", "science", 5, 5));
            var resp = new NavigationEngine(map, mission).Update(At(60, 60));
            Assert.Equal(10.0, resp.Layers.SlopeDeg, 1);
            Assert.Equal(0, resp.Layers.HazardClass);
            Assert.InRange(resp.Layers.ElevationM, 0, k * 118 + 1);
        }

        [Fact]
        public void Sessions_AreIsolated()
        {
            var map = TestTerrain.Flat(60, 2.0);
            var mission = SimpleMission(map, ("P1", "science", 20, 20), ("P2", "science", 80, 80));
            var engine = new NavigationEngine(map, mission);

            engine.Update(At(20, 20, session: "a")); // session "a" completes P1
            var b = engine.Update(At(10, 10, session: "b")); // session "b" did not complete it
            Assert.Equal("P1", b.NextWaypoint.Id);
            Assert.Equal("active", b.Objectives.First(o => o.Id == "P1").Status);

            var a = engine.Update(At(20, 20, session: "a"));
            Assert.Equal("P2", a.NextWaypoint.Id);
            Assert.Equal("done", a.Objectives.First(o => o.Id == "P1").Status);
        }

        [Fact]
        public void PlanRoute_DirectQueryWorks()
        {
            var map = TestTerrain.Flat(40, 2.0);
            var engine = new NavigationEngine(map, SimpleMission(map, ("P1", "science", 5, 5)));
            var route = engine.PlanRoute(new Point2(0, 0), new Point2(10, 0));
            Assert.True(route.Found);
            Assert.Equal(10.0, route.HorizontalLengthM, 6);
        }

        [Fact]
        public void ComputeTime_IsReported()
        {
            var map = TestTerrain.Flat(40, 2.0);
            var resp = new NavigationEngine(map, SimpleMission(map, ("P1", "science", 5, 5))).Update(At(10, 10));
            Assert.True(resp.ComputeMs >= 0);
        }
    }

    public class NavigationServiceTests
    {
        private sealed class FakeSensor : IAstronautSensor
        {
            public NavigationRequest Next { get; set; } = new NavigationRequest { SessionId = "x", X = 10, Z = 10 };
            public NavigationRequest Read() => Next;
        }

        private sealed class CaptureSink : IHudSink
        {
            public NavigationResponse Last { get; private set; }
            public int Count { get; private set; }
            public void Publish(NavigationResponse response) { Last = response; Count++; }
        }

        private sealed class FakeWeather : ISpaceWeatherProvider
        {
            public SpaceWeather Current() => new SpaceWeather { StormActive = true, Level = "G1" };
        }

        [Fact]
        public void Step_ReadsEngineAndPublishes()
        {
            var map = TestTerrain.Flat(40, 2.0);
            var mission = new MissionDefinition("m", "o",
                new[] { new Waypoint { Id = "P1", Position = new Point2(60, 60), IsObjective = true } },
                new[] { "P1" }, new HazardZone[0]);
            var sensor = new FakeSensor();
            var sink = new CaptureSink();
            var service = new NavigationService(new NavigationEngine(map, mission), sensor, sink, new FakeWeather());

            var resp = service.Step();
            Assert.Equal(1, sink.Count);
            Assert.Same(resp, sink.Last);
            Assert.Contains(resp.Alerts, a => a.Code == "SOLAR_STORM");
        }

        [Fact]
        public void Step_WorksWithoutWeatherOrSink()
        {
            var map = TestTerrain.Flat(40, 2.0);
            var mission = new MissionDefinition("m", "o",
                new[] { new Waypoint { Id = "P1", Position = new Point2(60, 60), IsObjective = true } },
                new[] { "P1" }, new HazardZone[0]);
            var service = new NavigationService(new NavigationEngine(map, mission), new FakeSensor(), null, null);
            Assert.NotNull(service.Step());
        }
    }

    public class ContractTests
    {
        [Fact]
        public void Response_SerializesWithCamelCaseContract()
        {
            var map = TestTerrain.Flat(40, 2.0);
            var mission = new MissionDefinition("m", "o",
                new[] { new Waypoint { Id = "P1", Position = new Point2(60, 60), IsObjective = true, Name = "Uno" } },
                new[] { "P1" }, new HazardZone[0]);
            var resp = new NavigationEngine(map, mission).Update(new NavigationRequest { SessionId = "s", X = 10, Z = 10, HeadingDeg = 45 });

            string json = MarpsJson.Serialize(resp);
            Assert.Contains("\"nextWaypoint\"", json);
            Assert.Contains("\"relativeBearingDeg\"", json);
            Assert.Contains("\"objectives\"", json);
            Assert.Contains("\"layers\"", json);
            Assert.Contains("\"missionComplete\":false", json);
            Assert.DoesNotContain("null", json); // nada es null en esta respuesta
        }

        [Fact]
        public void Request_DeserializesAndDefaultsMissingFields()
        {
            var req = MarpsJson.Deserialize<NavigationRequest>("{\"x\": 12.5, \"z\": -3, \"headingDeg\": 90, \"campoDesconocido\": true}");
            Assert.Equal(12.5, req.X);
            Assert.Equal(-3, req.Z);
            Assert.Equal(90, req.HeadingDeg);
            Assert.Equal("default", req.SessionId);
            Assert.Null(req.ActiveWaypointId);
        }

        [Fact]
        public void Request_IsCaseInsensitive()
        {
            var req = MarpsJson.Deserialize<NavigationRequest>("{\"X\": 1, \"Z\": 2, \"SESSIONID\": \"abc\"}");
            Assert.Equal(1, req.X);
            Assert.Equal("abc", req.SessionId);
        }

        [Fact]
        public void Point2_SerializesAsXZ()
        {
            string json = MarpsJson.Serialize(new Point2(1, 2));
            Assert.Equal("{\"x\":1,\"z\":2}", json);
        }
    }
}
