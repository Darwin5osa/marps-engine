using System;
using System.IO;
using System.Linq;
using Marps.Engine.Geometry;
using Marps.Engine.Json;
using Marps.Engine.Mission;
using Marps.Engine.Terrain;
using Marps.Engine.Tests.Helpers;
using Xunit;

namespace Marps.Engine.Tests
{
    public class MissionTests
    {
        private static readonly TerrainMetadata Meta = new TerrainMetadata
        {
            Width = 1025, Height = 1025, OriginX = -900, OriginY = 2567800, SideM = 2000,
        };

        private const string MissionJson = @"{
          ""name"": ""test"", ""objective"": ""obj"",
          ""order"": [""B"", ""P1"", ""P2"", ""B""],
          ""places"": [
            {""id"":""B"",""type"":""base"",""name"":""Base"",""X"":-600,""Y"":2569500,""reason"":""r""},
            {""id"":""P1"",""type"":""science"",""name"":""P1"",""X"":-150,""Y"":2569500,""reason"":""r""},
            {""id"":""P2"",""type"":""science"",""name"":""P2"",""X"":550,""Y"":2569200,""reason"":""r""},
            {""id"":""A3"",""type"":""antenna"",""name"":""Mast"",""X"":300,""Y"":2567950,""reason"":""r""}
          ]}";

        private const string HazardsJson = @"[
          {""type"":""rough zone"",""area_m2"":1000,""center"":[177.4,2569139.1],
           ""box_ULX_ULY_LRX_LRY"":[-212.2,2569799.0,658.0,2568212.7],""max_slope_deg"":45.4,""fraction_over_15"":0.309},
          {""type"":""slope >22°"",""area_m2"":10,""center"":[0,2567900],
           ""box_ULX_ULY_LRX_LRY"":[-10,2567910,10,2567890],""max_slope_deg"":18.0,""fraction_over_15"":0.1}
        ]";

        private static string WriteTemp(string name, string content)
        {
            string dir = Path.Combine(Path.GetTempPath(), "marps-mission-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(dir);
            string path = Path.Combine(dir, name);
            File.WriteAllText(path, content);
            return path;
        }

        [Fact]
        public void Load_ConvertsProjectionToLocalAndKeepsObjectiveOrder()
        {
            string missionPath = WriteTemp("mission.json", MissionJson);
            string hazardsPath = WriteTemp("hazards.json", HazardsJson);
            var m = MissionLoader.Load(missionPath, hazardsPath, Meta);

            Assert.Equal("test", m.Name);
            Assert.Equal(new[] { "B", "P1", "P2", "B" }, m.ObjectiveIds); // the base appears on departure and on return
            Assert.Equal(new Point2(300, 1700), m.Find("B").Position);
            Assert.Equal(new Point2(1450, 1400), m.Find("P2").Position);
            Assert.True(m.Find("B").IsObjective);
            Assert.False(m.Find("A3").IsObjective);
        }

        [Fact]
        public void Antennas_IncludeBaseAndMasts()
        {
            var m = MissionLoader.Load(WriteTemp("mission.json", MissionJson), WriteTemp("hazards.json", "[]"), Meta);
            var ids = m.Antennas.Select(a => a.Id).ToList();
            Assert.Contains("B", ids);
            Assert.Contains("A3", ids);
            Assert.DoesNotContain("P1", ids);
        }

        [Fact]
        public void HazardZone_BoxIsConvertedAndOrdered()
        {
            var zones = MissionLoader.LoadZones(WriteTemp("hazards.json", HazardsJson), Meta);
            Assert.Equal(2, zones.Count);
            var z = zones[0];
            Assert.Equal(-212.2 + 900, z.BoxMin.X, 6);
            Assert.Equal(2568212.7 - 2567800, z.BoxMin.Z, 6);
            Assert.Equal(658.0 + 900, z.BoxMax.X, 6);
            Assert.Equal(2569799.0 - 2567800, z.BoxMax.Z, 6);
            Assert.True(z.BoxMin.X < z.BoxMax.X && z.BoxMin.Z < z.BoxMax.Z);
        }

        [Theory]
        [InlineData(300, 300, 0)]     // inside the box
        [InlineData(100, 100, 0)]     // dentro
        [InlineData(0, 0, 0)]         // edge at the box corner
        public void HazardZone_DistanceIsZeroInsideBox(double x, double z, double expected)
        {
            var zone = new HazardZone { BoxMin = new Point2(0, 0), BoxMax = new Point2(400, 400) };
            Assert.Equal(expected, zone.DistanceTo(new Point2(x, z)), 6);
        }

        [Fact]
        public void HazardZone_DistanceOutsideBox_IsEuclidean()
        {
            var zone = new HazardZone { BoxMin = new Point2(0, 0), BoxMax = new Point2(10, 10) };
            Assert.Equal(5.0, zone.DistanceTo(new Point2(13, 14)), 6);
        }

        [Theory]
        [InlineData(44.0, "high")]
        [InlineData(22.5, "high")]
        [InlineData(22.0, "moderate")]
        [InlineData(10.0, "moderate")]
        public void HazardZone_SeverityFollowsMaxSlope(double maxSlope, string expected)
        {
            Assert.Equal(expected, new HazardZone { MaxSlopeDeg = maxSlope }.Severity);
        }

        [Fact]
        public void Mission_RejectsObjectiveNotInPlaces()
        {
            var wps = new[] { new Waypoint { Id = "B", Position = new Point2(0, 0) } };
            Assert.Throws<ArgumentException>(() => new MissionDefinition("n", "o", wps, new[] { "B", "XX" }, null));
        }

        [Fact]
        public void Mission_FindUnknownReturnsNull()
        {
            var wps = new[] { new Waypoint { Id = "B", Position = new Point2(0, 0) } };
            var m = new MissionDefinition("n", "o", wps, new[] { "B" }, null);
            Assert.Null(m.Find("ZZ"));
            Assert.Empty(m.HazardZones);
        }

        [Fact]
        public void LoadingMissingZonesFile_GivesEmptyList()
        {
            var m = MissionLoader.Load(WriteTemp("mission.json", MissionJson), Path.Combine(Path.GetTempPath(), "no-existe.json"), Meta);
            Assert.Empty(m.HazardZones);
        }

        [Fact]
        public void MetadataJson_RoundTripsWithProjectionFields()
        {
            string json = MarpsJson.Serialize(Meta);
            var back = MarpsJson.Deserialize<TerrainMetadata>(json);
            Assert.Equal(Meta.OriginX, back.OriginX);
            Assert.Equal(Meta.OriginY, back.OriginY);
            Assert.Equal(Meta.Width, back.Width);
            Assert.Contains("\"originX\"", json);
        }
    }
}
