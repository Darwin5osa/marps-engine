using System.Collections.Generic;
using Marps.Engine.Geometry;
using Marps.Engine.Routing;
using Marps.Engine.Tests.Helpers;
using Xunit;

namespace Marps.Engine.Tests
{
    public class WalkingModelTests
    {
        [Fact]
        public void FlatWalk_UsesFlatCoefficient()
        {
            Assert.Equal(100 * WalkingModel.Fitted.FlatMinPerM, WalkingModel.Fitted.EdgeMinutes(100, 0), 9);
        }

        [Fact]
        public void Climb_IsCostlierThanDescentOfSameHeight()
        {
            double up = WalkingModel.Fitted.EdgeMinutes(0, 10);
            double down = WalkingModel.Fitted.EdgeMinutes(0, -10);
            Assert.True(up > down);
            Assert.Equal(10 * WalkingModel.Fitted.ClimbMinPerM, up, 9);
            Assert.Equal(10 * WalkingModel.Fitted.DescentMinPerM, down, 9);
        }

        [Theory]
        [InlineData(100, 0)]
        [InlineData(50, 20)]
        [InlineData(10, -5)]
        public void LowerBound_NeverOverestimatesEdgeCost(double horizontal, double dh)
        {
            double edge = WalkingModel.Fitted.EdgeMinutes(horizontal, dh);
            double bound = WalkingModel.Fitted.LowerBoundMinutes(horizontal);
            Assert.True(bound <= edge + 1e-12);
        }

        [Fact]
        public void Fitted_ReproducesReferenceLegs_WithinTenthOfMinute()
        {
            // The 5 legs of reference_routes.json were used to fit the model: this checks the fit, not an external validation.
            foreach (var leg in ReferenceData.Legs)
            {
                double minutes = WalkingModel.Fitted.FlatMinPerM * leg.LengthM
                                 + WalkingModel.Fitted.ClimbMinPerM * leg.AscentM
                                 + WalkingModel.Fitted.DescentMinPerM * leg.DescentM;
                Assert.True(System.Math.Abs(minutes - leg.Minutes) < 0.12, $"{leg.Name}: {minutes:0.00} vs {leg.Minutes}");
            }
        }
    }

    public class PathSimplifierTests
    {
        private static List<Point2> Line(params (double X, double Z)[] pts)
        {
            var list = new List<Point2>();
            foreach (var p in pts) list.Add(new Point2(p.X, p.Z));
            return list;
        }

        [Fact]
        public void CollinearPoints_CollapseToEndpoints()
        {
            var pts = Line((0, 0), (1, 0), (2, 0), (3, 0), (4, 0));
            var s = PathSimplifier.Simplify(pts, 0.1);
            Assert.Equal(2, s.Count);
            Assert.Equal(pts[0], s[0]);
            Assert.Equal(pts[4], s[1]);
        }

        [Fact]
        public void Corner_IsKept()
        {
            var pts = Line((0, 0), (5, 0), (5, 5));
            var s = PathSimplifier.Simplify(pts, 0.5);
            Assert.Equal(3, s.Count);
        }

        [Theory]
        [InlineData(0.3, 2)]   // 0.3 m bump within 0.5 tolerance: discarded
        [InlineData(2.0, 3)]   // 2 m bump: kept
        public void Tolerance_DecidesWhatIsKept(double bump, int expectedCount)
        {
            var pts = Line((0, 0), (5, bump), (10, 0));
            var s = PathSimplifier.Simplify(pts, 0.5);
            Assert.Equal(expectedCount, s.Count);
        }

        [Fact]
        public void TwoOrFewerPoints_AreReturnedAsIs()
        {
            Assert.Empty(PathSimplifier.Simplify(new List<Point2>(), 1));
            Assert.Single(PathSimplifier.Simplify(Line((1, 1)), 1));
            Assert.Equal(2, PathSimplifier.Simplify(Line((0, 0), (9, 9)), 1).Count);
        }

        [Theory]
        [InlineData(5, 5, 0, 0, 10, 0, 5)]     // perpendicular al centro
        [InlineData(-3, 4, 0, 0, 10, 0, 5)]    // before the start: distance to the endpoint
        [InlineData(13, 4, 0, 0, 10, 0, 5)]    // after the end
        [InlineData(3, 0, 3, 3, 3, 3, 3)]      // segmento degenerado (un punto)
        public void DistanceToSegment_HandlesEndsAndDegenerate(double px, double pz, double ax, double az, double bx, double bz, double expected)
        {
            double d = PathSimplifier.DistanceToSegment(new Point2(px, pz), new Point2(ax, az), new Point2(bx, bz));
            Assert.Equal(expected, d, 6);
        }

        [Fact]
        public void RealRoute_SimplifiedPolylineStaysWithinTolerance()
        {
            // Smooth zigzag line generated with a sine: each discarded point must lie within <= tolerance.
            var pts = new List<Point2>();
            for (int i = 0; i <= 200; i++) pts.Add(new Point2(i, 0.8 * System.Math.Sin(i / 9.0)));

            double tol = 0.5;
            var s = PathSimplifier.Simplify(pts, tol);
            Assert.True(s.Count < pts.Count);

            foreach (var p in pts)
            {
                double best = double.MaxValue;
                for (int i = 1; i < s.Count; i++)
                    best = System.Math.Min(best, PathSimplifier.DistanceToSegment(p, s[i - 1], s[i]));
                Assert.True(best <= tol + 1e-9);
            }
        }
    }
}
