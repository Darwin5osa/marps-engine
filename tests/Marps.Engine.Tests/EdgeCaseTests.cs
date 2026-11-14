using System.Diagnostics;
using Marps.Engine.Geometry;
using Marps.Engine.Mission;
using Marps.Engine.Navigation;
using Marps.Engine.Terrain;
using Marps.Engine.Tests.Helpers;
using Xunit;

namespace Marps.Engine.Tests
{
    /// <summary>Inputs at the limits of the numeric range must not make the engine slow.</summary>
    public class EdgeCaseTests
    {
        private static MissionDefinition OneTarget() => new MissionDefinition(
            "edge", "edge",
            new[] { new Waypoint { Id = "P1", Type = "science", Name = "P1", Position = new Point2(60, 60) } },
            new[] { "P1" },
            null);

        [Theory]
        [InlineData(double.PositiveInfinity)]
        [InlineData(double.NegativeInfinity)]
        [InlineData(-1e12)]
        [InlineData(1e12)]
        public void LineOfSight_FarAwayPoint_ReturnsQuickly(double coord)
        {
            TerrainMap map = TestTerrain.Build(41, 2.0, (x, z) => 0.0);
            var sw = Stopwatch.StartNew();
            LineOfSight.Clear(map, new Point2(coord, coord), 1.7, new Point2(10, 10), 3.0);
            Assert.True(sw.ElapsedMilliseconds < 1000, $"took {sw.ElapsedMilliseconds} ms");
        }

        [Theory]
        [InlineData(double.PositiveInfinity)]
        [InlineData(-1e12)]
        public void Update_FarOffMapPosition_CompletesQuickly(double coord)
        {
            TerrainMap map = TestTerrain.Build(41, 2.0, (x, z) => 0.0);
            var engine = new NavigationEngine(map, OneTarget());
            var sw = Stopwatch.StartNew();
            var resp = engine.Update(new NavigationRequest { SessionId = "far", X = 10, Z = coord, HeadingDeg = 90 });
            Assert.True(sw.ElapsedMilliseconds < 2000, $"took {sw.ElapsedMilliseconds} ms");
            Assert.False(resp.Position.Inside);
        }
    }
}
