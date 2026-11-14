using System;
using System.IO;
using Marps.Engine.Geometry;
using Marps.Engine.Json;
using Marps.Engine.Terrain;
using Marps.Engine.Tests.Helpers;
using Xunit;

namespace Marps.Engine.Tests
{
    public class TerrainTests
    {
        [Fact]
        public void NodePosition_RowZeroIsNorth()
        {
            var map = TestTerrain.Flat(5, cell: 10);
            Assert.Equal(new Point2(0, 0), map.NodePosition(0, 4));   // esquina SO
            Assert.Equal(new Point2(40, 40), map.NodePosition(4, 0)); // esquina NE
            Assert.Equal(new Point2(0, 40), map.NodePosition(0, 0));  // esquina NO
        }

        [Theory]
        [InlineData(0, 0, 0)]
        [InlineData(3, 0, 3)]
        [InlineData(0, 1, 5)]
        [InlineData(4, 4, 24)]
        public void Index_IsRowMajor(int c, int r, int expected)
        {
            var map = TestTerrain.Flat(5);
            Assert.Equal(expected, map.Index(c, r));
        }

        [Fact]
        public void ElevationAt_OnNodeReturnsNodeHeight()
        {
            var map = TestTerrain.Build(5, 10, (x, z) => x + 2 * z);
            var p = map.NodePosition(2, 1);
            Assert.Equal(p.X + 2 * p.Z, map.ElevationAt(p), 3);
        }

        [Fact]
        public void ElevationAt_InterpolatesBilinearly()
        {
            // Inclined plane: bilinear interpolation is exact on a plane.
            var map = TestTerrain.Build(5, 10, (x, z) => 0.5 * x + 0.25 * z);
            var p = new Point2(15, 25);
            Assert.Equal(0.5 * 15 + 0.25 * 25, map.ElevationAt(p), 3);
        }

        [Fact]
        public void ElevationAt_OutsideClampsToBorder()
        {
            var map = TestTerrain.Build(5, 10, (x, z) => x);
            Assert.Equal(map.ElevationAt(new Point2(0, 0)), map.ElevationAt(new Point2(-100, -100)), 6);
            Assert.Equal(40.0, map.ElevationAt(new Point2(500, 20)), 3);
        }

        [Theory]
        [InlineData(0, 0, true)]
        [InlineData(40, 40, true)]
        [InlineData(20.5, 20.5, true)]
        [InlineData(-0.1, 10, false)]
        [InlineData(10, 40.1, false)]
        public void IsInside_DetectsBounds(double x, double z, bool expected)
        {
            var map = TestTerrain.Flat(5, cell: 10);
            Assert.Equal(expected, map.IsInside(new Point2(x, z)));
        }

        [Theory]
        // 5x5 grid, cell 10: row r has z = (4 - r) * 10
        [InlineData(12, 8, 1, 3)]      // c = round(1.2) = 1; r = round(4 - 0.8) = round(3.2) = 3
        [InlineData(-50, -50, 0, 4)]   // outside to the SW: clamped to the corner
        [InlineData(999, 999, 4, 0)]   // outside to the NE: clamped to the corner
        public void NearestNode_ClampsAndRounds(double x, double z, int expectedC, int expectedR)
        {
            var map = TestTerrain.Flat(5, cell: 10);
            map.NearestNode(new Point2(x, z), out int c, out int r);
            Assert.Equal(expectedC, c);
            Assert.Equal(expectedR, r);
        }

        [Fact]
        public void Slope_OfFlatTerrainIsZero()
        {
            var map = TestTerrain.Flat(6);
            foreach (var s in map.SlopeDeg) Assert.Equal(0f, s, 4);
        }

        [Theory]
        [InlineData(10.0)]
        [InlineData(30.0)]
        [InlineData(44.0)]
        public void Slope_OfRampMatchesAngle(double degrees)
        {
            double k = Math.Tan(degrees * Math.PI / 180.0);
            var map = TestTerrain.Build(9, 2.0, (x, z) => k * x);
            // nodo interior
            int idx = map.Index(4, 4);
            Assert.Equal(degrees, map.SlopeDeg[idx], 2);
            // edge node (one-sided difference)
            Assert.Equal(degrees, map.SlopeDeg[map.Index(0, 4)], 2);
        }

        [Fact]
        public void Slope_UsesBothDirections()
        {
            double k = Math.Tan(30 * Math.PI / 180.0);
            var map = TestTerrain.Build(9, 2.0, (x, z) => k * z);
            Assert.Equal(30.0, map.SlopeDeg[map.Index(4, 4)], 2);
        }

        [Fact]
        public void Constructor_RejectsWrongHeightCount()
        {
            var meta = new TerrainMetadata { Width = 3, Height = 3, SideM = 4 };
            Assert.Throws<ArgumentException>(() => new TerrainMap(meta, new float[8], new byte[9]));
        }

        [Fact]
        public void Constructor_RejectsWrongHazardCount()
        {
            var meta = new TerrainMetadata { Width = 3, Height = 3, SideM = 4 };
            Assert.Throws<ArgumentException>(() => new TerrainMap(meta, new float[9], new byte[8]));
        }

        [Fact]
        public void Constructor_RejectsTinyGrid()
        {
            var meta = new TerrainMetadata { Width = 1, Height = 1, SideM = 4 };
            Assert.Throws<ArgumentException>(() => new TerrainMap(meta, new float[1], new byte[1]));
        }

        [Fact]
        public void HazardFromSlope_MatchesThresholdsOnSyntheticTerrain()
        {
            double k = Math.Tan(18 * Math.PI / 180.0);
            var map = TestTerrain.Build(9, 2.0, (x, z) => k * x);
            Assert.Equal(1, map.Hazard[map.Index(4, 4)]); // 18° -> clase 1
        }

        [Fact]
        public void TerrainFiles_RoundTripThroughDisk()
        {
            string dir = Path.Combine(Path.GetTempPath(), "marps-tests-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(dir);
            try
            {
                var source = TestTerrain.Build(4, 3.0, (x, z) => x * 1.5 - z * 0.25);
                var hazard = new byte[16];
                for (int i = 0; i < 16; i++) hazard[i] = (byte)(i % 4);
                var src2 = new TerrainMap(source.Meta, source.Heights, hazard);

                var heightBytes = new byte[src2.Heights.Length * 4];
                Buffer.BlockCopy(src2.Heights, 0, heightBytes, 0, heightBytes.Length);
                File.WriteAllBytes(Path.Combine(dir, "terrain.f32"), heightBytes);
                File.WriteAllBytes(Path.Combine(dir, "hazard.u8"), hazard);
                File.WriteAllText(Path.Combine(dir, "terrain.json"), MarpsJson.Serialize(src2.Meta));

                var loaded = TerrainFiles.Load(dir);
                Assert.Equal(src2.Width, loaded.Width);
                Assert.Equal(src2.Height, loaded.Height);
                Assert.Equal(src2.Cell, loaded.Cell, 9);
                for (int i = 0; i < 16; i++)
                {
                    Assert.Equal(src2.Heights[i], loaded.Heights[i]);
                    Assert.Equal(hazard[i], loaded.Hazard[i]);
                }
            }
            finally
            {
                Directory.Delete(dir, true);
            }
        }

        [Fact]
        public void TerrainFiles_WrongSizeIsRejected()
        {
            string dir = Path.Combine(Path.GetTempPath(), "marps-tests-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(dir);
            try
            {
                var meta = new TerrainMetadata { Width = 4, Height = 4, SideM = 3, HeightsFile = "terrain.f32", HazardFile = "hazard.u8" };
                File.WriteAllBytes(Path.Combine(dir, "terrain.f32"), new byte[10]); // should be 64
                File.WriteAllBytes(Path.Combine(dir, "hazard.u8"), new byte[16]);
                Assert.Throws<InvalidDataException>(() => TerrainFiles.Load(meta, dir));
            }
            finally
            {
                Directory.Delete(dir, true);
            }
        }

        [Fact]
        public void TerrainFiles_MissingFileIsReported()
        {
            var meta = new TerrainMetadata { Width = 2, Height = 2, SideM = 1 };
            Assert.Throws<FileNotFoundException>(() => TerrainFiles.Load(meta, Path.GetTempPath() + "no-existe-" + Guid.NewGuid()));
        }

        [Fact]
        public void FileTerrainProvider_LoadsFromDirectory()
        {
            string dir = Path.Combine(Path.GetTempPath(), "marps-tests-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(dir);
            try
            {
                var src = TestTerrain.Flat(3, 1.0);
                var hb = new byte[src.Heights.Length * 4];
                Buffer.BlockCopy(src.Heights, 0, hb, 0, hb.Length);
                File.WriteAllBytes(Path.Combine(dir, "terrain.f32"), hb);
                File.WriteAllBytes(Path.Combine(dir, "hazard.u8"), src.Hazard);
                File.WriteAllText(Path.Combine(dir, "terrain.json"), MarpsJson.Serialize(src.Meta));

                TerrainMap map = new FileTerrainProvider(dir).Load();
                Assert.Equal(3, map.Width);
            }
            finally
            {
                Directory.Delete(dir, true);
            }
        }
    }
}
