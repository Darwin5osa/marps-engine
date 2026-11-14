using System;
using System.Diagnostics;
using System.Threading;
using Marps.Engine.Geometry;
using Marps.Engine.Mission;
using Marps.Engine.Navigation;
using Marps.Engine.Terrain;
using Marps.Engine.Tests.Helpers;
using Xunit;

namespace Marps.Engine.Tests
{
    public class BackgroundReplanTests
    {
        private static MissionDefinition OneTarget() => new MissionDefinition(
            "background", "background",
            new[] { new Waypoint { Id = "P1", Type = "science", Name = "P1", Position = new Point2(60, 60) } },
            new[] { "P1" },
            null);

        [Fact]
        public void BackgroundReplan_ReturnsPendingFirst_ThenTheRoute()
        {
            TerrainMap map = TestTerrain.Flat(41, 2.0);
            using var engine = new NavigationEngine(map, OneTarget(), new NavigationOptions { BackgroundReplanning = true });
            var req = new NavigationRequest { SessionId = "bg", X = 10, Z = 10, HeadingDeg = 45 };

            var first = engine.Update(req);
            Assert.NotNull(first.Route);
            Assert.True(first.Route.Pending);
            Assert.False(first.Route.Found);      // no previous route yet
            Assert.DoesNotContain(first.Alerts, a => a.Code == "NO_ROUTE");

            NavigationResponse latest = first;
            var clock = Stopwatch.StartNew();
            while (latest.Route.Pending && clock.ElapsedMilliseconds < 5000)
            {
                Thread.Sleep(5);
                latest = engine.Update(req);
            }

            Assert.False(latest.Route.Pending);
            Assert.True(latest.Route.Found);
            Assert.True(latest.Route.Replanned);
            Assert.Equal("P1", latest.Route.Target);
        }

        [Fact]
        public void BackgroundReplan_KeepsPreviousRouteWhileNewOneIsPending()
        {
            TerrainMap map = TestTerrain.Flat(41, 2.0);
            using var engine = new NavigationEngine(map, OneTarget(), new NavigationOptions { BackgroundReplanning = true });
            var req = new NavigationRequest { SessionId = "bg2", X = 10, Z = 10 };
            NavigationResponse latest = engine.Update(req);
            var clock = Stopwatch.StartNew();
            while (latest.Route.Pending && clock.ElapsedMilliseconds < 5000)
            {
                Thread.Sleep(5);
                latest = engine.Update(req);
            }
            Assert.True(latest.Route.Found);

            // Force a replan from a new position: the old route stays visible while the search runs.
            var moved = engine.Update(new NavigationRequest { SessionId = "bg2", X = 20, Z = 10, ForceReplan = true });
            Assert.True(moved.Route.Found);
            Assert.True(moved.Route.Pending || moved.Route.Replanned);
        }

        [Fact]
        public void Dispose_CanBeCalledTwice()
        {
            TerrainMap map = TestTerrain.Flat(41, 2.0);
            var engine = new NavigationEngine(map, OneTarget(), new NavigationOptions { BackgroundReplanning = true });
            engine.Dispose();
            engine.Dispose();
        }
    }
}
