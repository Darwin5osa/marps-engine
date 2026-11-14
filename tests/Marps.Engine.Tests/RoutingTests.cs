using System;
using System.Collections.Generic;
using System.Linq;
using Marps.Engine.Geometry;
using Marps.Engine.Routing;
using Marps.Engine.Terrain;
using Marps.Engine.Tests.Helpers;
using Xunit;

namespace Marps.Engine.Tests
{
    public class RoutingTests
    {
        private static readonly double Flat = WalkingModel.Fitted.FlatMinPerM;

        private static int CountHazard(TerrainMap map, RouteResult route, byte value)
        {
            int n = 0;
            foreach (var p in route.Points)
            {
                map.NearestNode(p, out int c, out int r);
                if (map.Hazard[map.Index(c, r)] == value) n++;
            }
            return n;
        }

        [Fact]
        public void Flat_DiagonalCorner_IsOptimalOctileDistance()
        {
            var map = TestTerrain.Flat(50, cell: 2.0);
            var router = new AStarRouter(map);
            var route = router.FindNodes(0, 49, 49, 0); // SO -> NE (fila 49 = sur)

            Assert.True(route.Found);
            double expected = 49 * 2.0 * Math.Sqrt(2.0);
            // EstimatedMinutes is stored as float (4 bytes per node): tolerance of 1e-4 min.
            Assert.Equal(expected, route.HorizontalLengthM, 6);
            Assert.Equal(expected * Flat, route.EstimatedMinutes, 4);
        }

        [Fact]
        public void Flat_StraightLine_IsExact()
        {
            var map = TestTerrain.Flat(30, cell: 2.0);
            var route = new AStarRouter(map).FindNodes(0, 29, 0, 0); // south to north
            Assert.True(route.Found);
            Assert.Equal(58.0, route.HorizontalLengthM, 6);
            Assert.Equal(58.0 * Flat, route.EstimatedMinutes, 6);
            Assert.Equal(0.0, route.AscentM, 9);
            Assert.Equal(0.0, route.DescentM, 9);
        }

        [Fact]
        public void SameStartAndGoal_ReturnsSingleNode()
        {
            var map = TestTerrain.Flat(10);
            var route = new AStarRouter(map).FindNodes(3, 3, 3, 3);
            Assert.True(route.Found);
            Assert.Single(route.Points);
            Assert.Equal(0.0, route.HorizontalLengthM);
            Assert.Equal(0.0, route.EstimatedMinutes);
        }

        [Fact]
        public void NodeOutsideMap_Throws()
        {
            var map = TestTerrain.Flat(10);
            Assert.Throws<ArgumentOutOfRangeException>(() => new AStarRouter(map).FindNodes(0, 0, 10, 0));
        }

        [Fact]
        public void Cliff_WithGap_RoutesThroughGap()
        {
            // 100 m ridge at x >= 60 except for a gap at z 40-50. Slopes > 22° are blocked.
            // The target is inside the gap (z 45), on the other side of the ridge.
            var map = TestTerrain.Build(60, 2.0, (x, z) => (x >= 60 && !(z >= 40 && z <= 50)) ? 100.0 : 0.0);
            var route = new AStarRouter(map).Find(new Point2(20, 10), new Point2(100, 45));

            Assert.True(route.Found);
            Assert.Contains(route.Points, p => p.X >= 58 && p.X <= 62 && p.Z >= 38 && p.Z <= 52);
            Assert.True(route.MaxEdgeSlopeDeg <= 22.0 + 1e-6);
        }

        [Fact]
        public void Cliff_WithoutGap_IsUnreachable()
        {
            var map = TestTerrain.Build(60, 2.0, (x, z) => x >= 60 ? 100.0 : 0.0);
            var route = new AStarRouter(map).Find(new Point2(20, 10), new Point2(100, 10));
            Assert.False(route.Found);
            Assert.Empty(route.Points);
        }

        [Fact]
        public void HazardTwo_IsNeverEntered_EvenIfFlat()
        {
            // Flat terrain with a column of hazard 2 and a gap.
            var flat = TestTerrain.Flat(40, 2.0);
            var map = TestTerrain.WithHazard(flat, (c, r) => (c == 20 && (r < 15 || r > 25)) ? (byte)2 : (byte)0);
            var route = new AStarRouter(map).FindNodes(0, 20, 39, 20);

            Assert.True(route.Found);
            Assert.Equal(0, CountHazard(map, route, 2));
            // Must cross column 20 inside the gap (rows 15-25).
            Assert.Contains(route.Points, p => Math.Abs(p.X - 40) < 1e-6 && p.Z <= (39 - 15) * 2.0 && p.Z >= (39 - 25) * 2.0);
        }

        [Fact]
        public void RoughZone_IsAvoidedWhenFactorIsHigh()
        {
            // Rough strip (hazard 3) from column 15 to 25, except the edge rows (free passage top and bottom).
            var flat = TestTerrain.Flat(40, 2.0);
            var map = TestTerrain.WithHazard(flat, (c, r) => (c >= 15 && c <= 25 && r >= 1 && r <= 38) ? (byte)3 : (byte)0);

            var cheap = new AStarRouter(map, null, new RouteOptions { RoughFactor = 1.0 }).FindNodes(0, 20, 39, 20);
            var dear = new AStarRouter(map, null, new RouteOptions { RoughFactor = 50.0 }).FindNodes(0, 20, 39, 20);

            Assert.True(cheap.Found && dear.Found);
            Assert.True(CountHazard(map, cheap, 3) > 0, "With factor 1 it should cross the rough zone.");
            Assert.Equal(0, CountHazard(map, dear, 3));
            Assert.True(dear.EstimatedMinutes > cheap.EstimatedMinutes);
        }

        [Fact]
        public void Uphill_CostsMoreThanDownhill_OverSameDistance()
        {
            var up = TestTerrain.Build(41, 2.0, (x, z) => 0.2 * x);      // rises toward the east
            var rUp = new AStarRouter(up).FindNodes(0, 20, 40, 20);
            var rDown = new AStarRouter(up).FindNodes(40, 20, 0, 20);     // misma distancia, bajando

            Assert.True(rUp.Found && rDown.Found);
            Assert.Equal(rUp.HorizontalLengthM, rDown.HorizontalLengthM, 6);
            Assert.True(rUp.EstimatedMinutes > rDown.EstimatedMinutes);
            Assert.True(rUp.AscentM > 0 && rUp.DescentM < 1e-9);
            Assert.True(rDown.DescentM > 0 && rDown.AscentM < 1e-9);
        }

        [Fact]
        public void EstimatedMinutes_EqualsSumOfTimeModelOverSteps()
        {
            var map = TestTerrain.Build(41, 2.0, (x, z) => 0.2 * x + 0.1 * Math.Sin(z));
            var route = new AStarRouter(map).FindNodes(0, 20, 40, 5);
            Assert.True(route.Found);

            double expected = Flat * route.HorizontalLengthM
                              + WalkingModel.Fitted.ClimbMinPerM * route.AscentM
                              + WalkingModel.Fitted.DescentMinPerM * route.DescentM;
            Assert.Equal(expected, route.EstimatedMinutes, 3);
        }

        [Fact]
        public void RepeatedSearches_AreDeterministicAndIndependent()
        {
            // Max slope ~17°: the whole map is walkable, so b must find a route.
            var map = TestTerrain.Build(60, 2.0, (x, z) => 1.0 * Math.Sin(x / 7.0) + 0.8 * Math.Cos(z / 5.0));
            var router = new AStarRouter(map);

            var a1 = router.FindNodes(0, 0, 59, 59);
            var b = router.FindNodes(59, 0, 0, 59);
            var a2 = router.FindNodes(0, 0, 59, 59);

            Assert.Equal(a1.HorizontalLengthM, a2.HorizontalLengthM, 9);
            Assert.Equal(a1.EstimatedMinutes, a2.EstimatedMinutes, 9);
            Assert.Equal(a1.Points.Count, a2.Points.Count);
            Assert.True(b.Found);
        }

        [Fact]
        public void Find_UsesNearestNodesForWorldPoints()
        {
            var map = TestTerrain.Flat(20, cell: 10);
            var route = new AStarRouter(map).Find(new Point2(1, 1), new Point2(189, 189));
            Assert.True(route.Found);
            Assert.Equal(0, route.Points[0].X, 6);
            Assert.Equal(0, route.Points[0].Z, 6);
            Assert.Equal(190, route.Points[route.Points.Count - 1].X, 6);
        }

        [Fact]
        public void Route_PointsAreContiguousAndNeverExceedSlopeLimit_OnRandomTerrains()
        {
            var rng = new Random(20260514); // semilla fija: reproducible
            for (int trial = 0; trial < 12; trial++)
            {
                double a = 2 + rng.NextDouble() * 6;
                double b = 2 + rng.NextDouble() * 6;
                double ph = rng.NextDouble() * 6;
                var map = TestTerrain.Build(48, 2.0, (x, z) => a * Math.Sin(x / 6.0 + ph) * Math.Cos(z / 5.0) + b * Math.Sin((x + z) / 9.0));
                var router = new AStarRouter(map);

                for (int q = 0; q < 6; q++)
                {
                    int sc = rng.Next(48), sr = rng.Next(48), gc = rng.Next(48), gr = rng.Next(48);
                    var route = router.FindNodes(sc, sr, gc, gr);
                    if (!route.Found) continue;

                    Assert.True(route.MaxEdgeSlopeDeg <= 22.0 + 1e-6, $"slope {route.MaxEdgeSlopeDeg}");
                    Assert.Equal(0, CountHazard(map, route, 2));
                    for (int i = 1; i < route.Points.Count; i++)
                    {
                        double step = route.Points[i - 1].DistanceTo(route.Points[i]);
                        Assert.True(step <= 2.0 * Math.Sqrt(2.0) + 1e-6, "jump larger than a diagonal cell");
                    }
                }
            }
        }

        [Fact]
        public void LargeGrid_CornerToCorner_ComputesQuickly()
        {
            var map = TestTerrain.Build(257, 7.8125, (x, z) => 20 * Math.Sin(x / 80.0) * Math.Cos(z / 90.0));
            var router = new AStarRouter(map);
            var route = router.FindNodes(0, 256, 256, 0);

            Assert.True(route.Found);
            Assert.True(route.ComputeMs < 2000, $"took {route.ComputeMs:0.0} ms");
        }

        [Fact]
        public void FoundRoute_StartsAndEndsAtRequestedNodes()
        {
            var map = TestTerrain.Flat(25, 2.0);
            var route = new AStarRouter(map).FindNodes(2, 3, 20, 22);
            Assert.True(route.Found);
            Assert.Equal(map.NodePosition(2, 3), route.Points.First());
            Assert.Equal(map.NodePosition(20, 22), route.Points.Last());
        }

        [Fact]
        public void MinHeap_PopsInAscendingOrder()
        {
            var heap = new MinHeap(2);
            var rng = new Random(7);
            var keys = Enumerable.Range(0, 200).Select(_ => (float)rng.NextDouble()).ToList();
            for (int i = 0; i < keys.Count; i++) heap.Push(keys[i], i);

            float prev = float.MinValue;
            int popped = 0;
            while (heap.Count > 0)
            {
                int v = heap.Pop();
                float k = keys[v];
                Assert.True(k >= prev);
                prev = k;
                popped++;
            }
            Assert.Equal(200, popped);
        }
    }
}
