using Marps.Engine.Geometry;
using Marps.Engine.Terrain;
using Xunit;

namespace Marps.Engine.Tests
{
    public class GeometryTests
    {
        [Theory]
        [InlineData(0, 10, 0)]
        [InlineData(10, 0, 90)]
        [InlineData(0, -10, 180)]
        [InlineData(-10, 0, 270)]
        [InlineData(10, 10, 45)]
        [InlineData(-10, -10, 225)]
        public void Bearing_BetweenPoints_UsesNorthClockwise(double dx, double dz, double expected)
        {
            double b = Bearing.Between(new Point2(0, 0), new Point2(dx, dz));
            Assert.Equal(expected, b, 6);
        }

        [Theory]
        [InlineData(350, 10, -20)]   // target just left of the look direction
        [InlineData(10, 350, 20)]    // just right
        [InlineData(180, 0, 180)]
        [InlineData(90, 90, 0)]
        [InlineData(270, 0, -90)]
        [InlineData(0, 90, -90)]
        [InlineData(0, 720, 0)]
        [InlineData(-10, 0, -10)]
        public void Bearing_Relative_WrapsIntoMinus180To180(double bearing, double heading, double expected)
        {
            Assert.Equal(expected, Bearing.Relative(bearing, heading), 6);
        }

        [Theory]
        [InlineData(-1, 359)]
        [InlineData(360, 0)]
        [InlineData(725, 5)]
        public void Bearing_Normalize360_Wraps(double input, double expected)
        {
            Assert.Equal(expected, Bearing.Normalize360(input), 9);
        }

        [Fact]
        public void Point2_DistanceTo_IsEuclidean()
        {
            Assert.Equal(5.0, new Point2(0, 0).DistanceTo(new Point2(3, 4)), 9);
        }

        [Fact]
        public void Point2_ToString_IsReadable()
        {
            Assert.Equal("(1.50, -2.25)", new Point2(1.5, -2.25).ToString());
        }

        [Theory]
        [InlineData(-600, 2569500, 300, 1700)]   // base B
        [InlineData(-150, 2569500, 750, 1700)]   // P1
        [InlineData(550, 2569200, 1450, 1400)]   // P2
        [InlineData(-900, 2567800, 0, 0)]        // esquina suroeste
        [InlineData(1100, 2569800, 2000, 2000)]  // esquina noreste
        public void Projection_MapsToUnityLocalMeters(double X, double Y, double x, double z)
        {
            var meta = new TerrainMetadata { OriginX = -900, OriginY = 2567800, Width = 1025, Height = 1025, SideM = 2000 };
            var local = meta.ProjectionToLocal(X, Y);
            Assert.Equal(x, local.X, 6);
            Assert.Equal(z, local.Z, 6);
        }

        [Theory]
        [InlineData(0, 0)]
        [InlineData(1234.5, 678.9)]
        [InlineData(2000, 2000)]
        public void Projection_RoundTrips(double x, double z)
        {
            var meta = new TerrainMetadata { OriginX = -900, OriginY = 2567800 };
            var back = meta.LocalToProjection(new Point2(x, z));
            var local = meta.ProjectionToLocal(back.X, back.Y);
            Assert.Equal(x, local.X, 9);
            Assert.Equal(z, local.Z, 9);
        }

        [Theory]
        [InlineData(1001, 2.0)]
        [InlineData(1025, 1.953125)]   // what Unity and the engine use
        [InlineData(257, 7.8125)]
        public void CellSize_IsSideDividedByIntervals(int nodes, double expectedCell)
        {
            var meta = new TerrainMetadata { Width = nodes, Height = nodes, SideM = 2000 };
            Assert.Equal(expectedCell, meta.CellM, 6);
        }
    }
}
