using System;
using System.Linq;
using System.Threading.Tasks;
using Marps.Engine.Abstractions;
using Marps.Engine.Geometry;
using Marps.Engine.Navigation;
using Marps.Engine.Routing;
using Marps.Engine.Tests.Helpers;
using Xunit;
using Xunit.Abstractions;
using static Marps.Engine.Tests.Interface.Fixtures;

namespace Marps.Engine.Tests.Interface
{
    [Trait("Method", "GetCostField")]
    public class CostFieldEdgeCaseTests
    {
        private readonly ITestOutputHelper _out;

        public CostFieldEdgeCaseTests(ITestOutputHelper output)
        {
            _out = output;
        }

        /// <summary>Optimal router (heuristic weight 1) so its times are directly comparable with the field.</summary>
        private static INavigationEngine Exact(Terrain.TerrainMap map = null) =>
            NavigationEngineFactory.Create(map ?? Flat(), Square(), routeOptions: new RouteOptions { HeuristicWeight = 1.0 });

        private static Terrain.TerrainMap Hilly() =>
            TestTerrain.Build(Nodes, Cell, (x, z) => 6 * Math.Sin(x / 15.0) * Math.Cos(z / 20.0) + 0.05 * x);

        [Fact]
        public void TargetNode_IsZero_AndReachable()
        {
            var field = NewEngine().GetCostField("B");
            Assert.Equal(0.0, field.MinutesAt(BasePos));
            Assert.True(field.IsReachable(BasePos));
            Assert.Equal("B", field.TargetId);
            Assert.Equal(BasePos.X, field.Target.X);
        }

        [Fact]
        public void FlatGround_MatchesTheWalkingModel()
        {
            // 90 m due east on flat ground: 90 * 0.013937 min.
            var field = NewEngine().GetCostField("B");
            Assert.Equal(90 * WalkingModel.Fitted.FlatMinPerM, field.MinutesAt(P1Pos), 4);
        }

        [Theory]
        [InlineData(100, 10)]
        [InlineData(100, 100)]
        [InlineData(10, 100)]
        [InlineData(57, 33)]
        [InlineData(3, 111)]
        public void EqualsTheOptimalRouteTime_OnHillyTerrain(double x, double z)
        {
            var engine = Exact(Hilly());
            var field = engine.GetCostField("B");
            var route = engine.PlanRoute(new Point2(x, z), BasePos);
            Assert.True(route.Found);
            Assert.Equal(route.EstimatedMinutes, field.MinutesAt(new Point2(x, z)), 3);
        }

        [Fact]
        public void IsNeverAboveTheDefaultWeightedRoute()
        {
            var engine = NewEngine(map: Hilly());             // heuristic weight 1.25
            var field = engine.GetCostField("B");
            var from = new Point2(100, 100);
            var route = engine.PlanRoute(from, BasePos);
            Assert.True(field.MinutesAt(from) <= route.EstimatedMinutes + 1e-4);
            Assert.True(field.MinutesAt(from) >= route.EstimatedMinutes / 1.25 - 1e-4);
        }

        [Fact]
        public void MeasuresTheWalkTowardTheTarget_NotAway()
        {
            // A ramp rising to the east: walking from the low end up to the high end costs more than coming down.
            var ramp = TestTerrain.Build(Nodes, Cell, (x, z) => 0.2 * x);       // ~11°, walkable
            var engine = NewEngine(map: ramp, mission: Custom(new[] { Wp("LOW", "base", 10, 60), Wp("HIGH", "science", 100, 60) }, "LOW", "HIGH"));
            double up = engine.GetCostField("HIGH").MinutesAt(new Point2(10, 60));
            double down = engine.GetCostField("LOW").MinutesAt(new Point2(100, 60));
            Assert.True(up > down, $"uphill {up} should cost more than downhill {down}");
            var model = WalkingModel.Fitted;
            Assert.Equal(model.EdgeMinutes(90, 18), up, 2);
            Assert.Equal(model.EdgeMinutes(90, -18), down, 2);
        }

        [Fact]
        public void WalledTarget_LeavesEverythingOutsideUnreachable()
        {
            var field = NewEngine(map: Walled(P1Pos)).GetCostField("P1");
            Assert.False(field.IsReachable(BasePos));
            Assert.True(double.IsPositiveInfinity(field.MinutesAt(BasePos)));
            Assert.True(field.IsReachable(new Point2(P1Pos.X + 2, P1Pos.Z)));       // inside the wall
            Assert.True(field.ReachableNodes < Nodes * Nodes / 10);
        }

        [Fact]
        public void BlockedTargetNode_IsReachableOnlyFromItself()
        {
            var field = NewEngine(map: BlockedNode(P1Pos)).GetCostField("P1");
            Assert.Equal(0.0, field.MinutesAt(P1Pos));
            Assert.Equal(1, field.ReachableNodes);
            Assert.False(field.IsReachable(new Point2(P1Pos.X - 2, P1Pos.Z)));
        }

        [Fact]
        public void TooSteepTerrain_IsUnreachable()
        {
            // A 2 m cliff between x = 58 and x = 60: steeper than 22° in one step, so the east side is cut off.
            var cliff = TestTerrain.Build(Nodes, Cell, (x, z) => x >= 60 ? 5.0 : 0.0);
            var engine = NewEngine(map: TestTerrain.WithHazard(cliff, (c, r) => 0));
            var field = engine.GetCostField("B");
            Assert.False(field.IsReachable(P1Pos));
            Assert.False(engine.PlanRoute(P1Pos, BasePos).Found);              // agrees with the router
        }

        [Fact]
        public void RoughFactor_IsApplied()
        {
            var rough = TestTerrain.WithHazard(Flat(), (c, r) => 3);
            var plain = NavigationEngineFactory.Create(rough, Square()).GetCostField("B");
            var slow = NavigationEngineFactory.Create(rough, Square(), routeOptions: new RouteOptions { RoughFactor = 2.0 }).GetCostField("B");
            Assert.Equal(2 * plain.MinutesAt(P1Pos), slow.MinutesAt(P1Pos), 4);
        }

        [Theory]
        [InlineData(double.NaN, 10)]
        [InlineData(10, double.PositiveInfinity)]
        public void NonFiniteQuery_IsUnreachable(double x, double z)
        {
            var field = NewEngine().GetCostField("B");
            Assert.True(double.IsPositiveInfinity(field.MinutesAt(new Point2(x, z))));
            Assert.False(field.IsReachable(new Point2(x, z)));
            Assert.False(field.CanReachWithin(new Point2(x, z), 1e9));
        }

        [Fact]
        public void QueryOutsideTheMap_UsesTheNearestEdgeNode()
        {
            var field = NewEngine().GetCostField("B");
            Assert.Equal(field.MinutesAtNode(Nodes - 1, Nodes - 1), field.MinutesAt(new Point2(1e6, -1e6)));
            Assert.Equal(field.MinutesAtNode(0, 0), field.MinutesAt(new Point2(-1e6, 1e6)));
        }

        [Fact]
        public void CanReachWithin_ComparesWithTheBudget()
        {
            var field = NewEngine().GetCostField("B");
            double t = field.MinutesAt(P1Pos);
            Assert.True(field.CanReachWithin(P1Pos, t + 0.01));
            Assert.False(field.CanReachWithin(P1Pos, t - 0.01));
        }

        [Fact]
        public void MinutesAtNode_RejectsNodesOutsideTheGrid()
        {
            var field = NewEngine().GetCostField("B");
            Assert.Throws<ArgumentOutOfRangeException>(() => field.MinutesAtNode(-1, 0));
            Assert.Throws<ArgumentOutOfRangeException>(() => field.MinutesAtNode(0, Nodes));
        }

        [Fact]
        public void CopyMinutes_IsACopy()
        {
            var field = NewEngine().GetCostField("B");
            var copy = field.CopyMinutes();
            Assert.Equal(Nodes * Nodes, copy.Length);
            double before = field.MinutesAt(P1Pos);
            for (int i = 0; i < copy.Length; i++) copy[i] = -1;
            Assert.Equal(before, field.MinutesAt(P1Pos));
        }

        [Fact]
        public void PlaceFields_AreCachedPerPlace()
        {
            var engine = NewEngine();
            Assert.Same(engine.GetCostField("B"), engine.GetCostField("B"));
            Assert.NotSame(engine.GetCostField("B"), engine.GetCostField("P1"));
        }

        [Fact]
        public void ConcurrentCalls_ShareOneBuild()
        {
            var engine = NewEngine(map: Hilly());
            var fields = new CostField[16];
            Parallel.For(0, fields.Length, i => fields[i] = engine.GetCostField("P2"));
            Assert.All(fields, f => Assert.Same(fields[0], f));
        }

        [Fact]
        public void Fields_DoNotChangeSessions()
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));
            engine.GetCostField("B");
            var resp = engine.Update(At(BasePos));
            Assert.Equal("P1", resp.Route.Target);
            Assert.False(resp.Route.Replanned);
        }

        [Fact]
        public void UnknownOrNullPlace_Throws()
        {
            var engine = NewEngine();
            Assert.Throws<ArgumentNullException>(() => engine.GetCostField((string)null));
            Assert.Throws<ArgumentException>(() => engine.GetCostField("NOPE"));
            Assert.Throws<ArgumentException>(() => engine.GetCostField(""));
        }

        [Fact]
        public void PlaceThatIsNotAStop_IsAllowed()
        {
            var field = NewEngine().GetCostField("A1");       // antenna, not a mission step
            Assert.Equal(0.0, field.MinutesAt(new Point2(60, 60)));
        }

        [Fact]
        public void FreePoint_IsNotCached_AndHasNoId()
        {
            var engine = NewEngine();
            var a = engine.GetCostField(new Point2(50, 50));
            var b = engine.GetCostField(new Point2(50, 50));
            Assert.NotSame(a, b);
            Assert.Null(a.TargetId);
            Assert.Equal(0.0, a.MinutesAt(new Point2(50, 50)));
        }

        [Theory]
        [InlineData(double.NaN, 10)]
        [InlineData(10, double.NegativeInfinity)]
        [InlineData(-1, 10)]
        [InlineData(10, 500)]
        public void FreePoint_NonFiniteOrOffMap_Throws(double x, double z)
        {
            Assert.Throws<ArgumentException>(() => NewEngine().GetCostField(new Point2(x, z)));
        }

        [Fact]
        public void WorksAfterDispose()
        {
            var engine = NewEngine(options: new NavigationOptions { BackgroundReplanning = true });
            engine.Dispose();
            Assert.Equal(0.0, engine.GetCostField("B").MinutesAt(BasePos));
        }

        [SkippableFact]
        public void RealData_ReturnToBaseFromP1_MatchesTheReferenceLeg()
        {
            Skip.IfNot(RealData.Available, "No processed data.");
            var engine = NavigationEngineFactory.Create(RealData.Terrain, RealData.Mission);
            var field = engine.GetCostField(engine.Base.Id);
            var p1 = engine.Mission.Find("P1").Position;
            double minutes = field.MinutesAt(p1);
            var route = engine.PlanRoute(p1, engine.Base.Position);
            Assert.True(route.Found);
            Assert.InRange(minutes, route.EstimatedMinutes / 1.25, route.EstimatedMinutes + 1e-3);
            Assert.InRange(minutes, 5, 15);                 // the reference B <-> P1 leg is about 9 min
            _out.WriteLine($"field: {field.ComputeMs:0.0} ms, {field.ReachableNodes} reachable nodes; P1 -> B {minutes:0.00} min (route {route.EstimatedMinutes:0.00})");
        }
    }

    [Trait("Method", "SpeedAt")]
    public class SpeedAtEdgeCaseTests
    {
        [Fact]
        public void FlatGround_IsAbout1Point2MetersPerSecond()
        {
            double v = NewEngine().SpeedAt(0, uphill: true);
            Assert.Equal(1.0 / (0.013937 * 60), v, 6);
            Assert.InRange(v, 1.15, 1.25);
            Assert.Equal(v, NewEngine().SpeedAt(0, uphill: false), 9);
        }

        [Theory]
        [InlineData(5)]
        [InlineData(15)]
        [InlineData(22)]
        public void Uphill_IsSlowerThanDownhill_AndBothSlowerThanFlat(double slope)
        {
            var e = NewEngine();
            double flat = e.SpeedAt(0, true), up = e.SpeedAt(slope, true), down = e.SpeedAt(slope, false);
            Assert.True(up < down && down < flat, $"up {up}, down {down}, flat {flat}");
        }

        [Fact]
        public void IsTheInverseOfTheRouteTime()
        {
            // Walking a 100 m horizontal leg climbing 10 m at SpeedAt must take exactly the model's minutes.
            var e = NewEngine();
            double slope = Math.Atan(10.0 / 100.0) * 180 / Math.PI;
            double seconds = 100.0 / e.SpeedAt(slope, true);
            Assert.Equal(WalkingModel.Fitted.EdgeMinutes(100, 10), seconds / 60, 9);
        }

        [Fact]
        public void SpeedDecreasesAsTheSlopeGrows()
        {
            var e = NewEngine();
            double last = double.MaxValue;
            for (int s = 0; s < 89; s++)
            {
                double v = e.SpeedAt(s, true);
                Assert.True(v < last || s == 0);
                last = v;
            }
        }

        [Theory]
        [InlineData(-10, 10)]
        [InlineData(-30, 30)]
        public void SignOfTheSlopeIsIgnored(double negative, double positive)
        {
            var e = NewEngine();
            Assert.Equal(e.SpeedAt(positive, true), e.SpeedAt(negative, true), 12);
        }

        [Theory]
        [InlineData(90)]
        [InlineData(120)]
        [InlineData(double.PositiveInfinity)]
        public void VerticalOrBeyond_IsZero(double slope)
        {
            Assert.Equal(0.0, NewEngine().SpeedAt(slope, true));
        }

        [Fact]
        public void NaNSlope_CountsAsFlat()
        {
            var e = NewEngine();
            Assert.Equal(e.SpeedAt(0, true), e.SpeedAt(double.NaN, true));
        }

        [Fact]
        public void UsesTheEngineWalkingModel()
        {
            var slow = new WalkingModel(0.02, 0.1, 0.05);
            var e = NavigationEngineFactory.Create(Flat(), Square(), model: slow);
            Assert.Equal(1.0 / (0.02 * 60), e.SpeedAt(0, true), 9);
        }

        [Theory]
        [InlineData(0, 0.1, 0.05)]
        [InlineData(-0.01, 0.1, 0.05)]
        [InlineData(double.NaN, 0.1, 0.05)]
        [InlineData(0.01, -0.1, 0.05)]
        [InlineData(0.01, 0.1, double.PositiveInfinity)]
        public void InvalidWalkingModel_Throws(double flat, double climb, double descent)
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => new WalkingModel(flat, climb, descent));
        }
    }
}
