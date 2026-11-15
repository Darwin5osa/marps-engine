using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Marps.Engine.Abstractions;
using Marps.Engine.Data;
using Marps.Engine.Geometry;
using Marps.Engine.Mission;
using Marps.Engine.Navigation;
using Marps.Engine.Routing;
using Marps.Engine.Terrain;
using Xunit;
using static Marps.Engine.Tests.Interface.Fixtures;

namespace Marps.Engine.Tests.Interface
{
    /// <summary>The engine can only be reached through its interfaces, and nothing outside can change its data.</summary>
    [Trait("Method", "Encapsulation")]
    public class EncapsulationTests
    {
        private static readonly Assembly Core = typeof(INavigationEngine).Assembly;

        [Fact]
        public void Factory_ReturnsTheInterface()
        {
            using var engine = NavigationEngineFactory.Create(Flat(), Square());
            Assert.IsAssignableFrom<INavigationEngine>(engine);
            Assert.Equal("P1", engine.Update(At(BasePos)).Route.Target);
        }

        [Theory]
        [InlineData("Marps.Engine.Navigation.NavigationEngine")]
        [InlineData("Marps.Engine.Routing.AStarRouter")]
        [InlineData("Marps.Engine.Routing.PathSimplifier")]
        [InlineData("Marps.Engine.Routing.MinHeap")]
        [InlineData("Marps.Engine.Navigation.LineOfSight")]
        public void Implementation_IsNotPublic(string typeName)
        {
            var type = Core.GetType(typeName, throwOnError: true);
            Assert.False(type.IsPublic, $"{typeName} must not be public: frontends would bypass INavigationEngine.");
        }

        [Fact]
        public void PublicSurface_IsOnlyInterfacesDataAndTheFactory()
        {
            // Every public type is an interface, the factory, or a passive data / settings type.
            var allowed = new HashSet<string>
            {
                "INavigationEngine", "IAstronautSensor", "IHudSink", "ISpaceWeatherProvider", "ITerrainProvider",
                "NavigationEngineFactory", "NavigationService",
                "NavigationRequest", "NavigationResponse", "SpaceWeather", "EmergencyInfo", "PositionInfo", "RouteInfo",
                "NextWaypointInfo", "ObjectiveInfo", "HazardInfo", "AlertInfo", "LayerInfo", "AntennaInfo",
                "TripRequest", "TripStop", "TripPlan", "TripLeg", "ItineraryLeg", "RouteResult", "CostField",
                "NavigationOptions", "RouteOptions", "WalkingModel",
                "TerrainMap", "TerrainMetadata", "MissionDefinition", "Waypoint", "HazardZone",
                "Point2", "Bearing",
            };
            var unexpected = Core.GetExportedTypes().Select(t => t.Name).Where(n => !allowed.Contains(n)).ToList();
            Assert.True(unexpected.Count == 0, "Unexpected public types: " + string.Join(", ", unexpected));
        }

        [Fact]
        public void NoPublicConstructorOrMutator_ForCostField()
        {
            Assert.Empty(typeof(CostField).GetConstructors());
            Assert.All(typeof(CostField).GetProperties(), p => Assert.Null(p.SetMethod));
        }

        [Fact]
        public void Core_DoesNotDependOnJsonOrUnity()
        {
            var refs = Core.GetReferencedAssemblies().Select(a => a.Name).ToList();
            Assert.DoesNotContain(refs, n => n.StartsWith("System.Text.Json", StringComparison.Ordinal));
            Assert.DoesNotContain(refs, n => n.StartsWith("Unity", StringComparison.Ordinal));
        }

        [Fact]
        public void Loaders_LiveInTheDataAssembly()
        {
            Assert.NotEqual(Core, typeof(TerrainFiles).Assembly);
            Assert.Equal(typeof(TerrainFiles).Assembly, typeof(MissionLoader).Assembly);
            Assert.Equal(typeof(TerrainFiles).Assembly, typeof(MarpsJson).Assembly);
            Assert.IsAssignableFrom<ITerrainProvider>(new FileTerrainProvider("x"));
        }

        [Fact]
        public void TerrainGrids_AreNotPubliclyWritable()
        {
            foreach (var name in new[] { "Heights", "Hazard", "SlopeDeg" })
            {
                var prop = typeof(TerrainMap).GetProperty(name, BindingFlags.Instance | BindingFlags.Public);
                Assert.Null(prop);
            }
        }
    }

    [Trait("Method", "Map / Mission / Base")]
    public class ImmutabilityTests
    {
        [Fact]
        public void TerrainMap_CopiesItsInputArrays()
        {
            var meta = new TerrainMetadata { Width = 3, Height = 3, SideM = 4 };
            var heights = new float[9];
            var hazard = new byte[9];
            var map = new TerrainMap(meta, heights, hazard);
            heights[4] = 100;
            hazard[4] = 2;
            Assert.Equal(0f, map.HeightAt(1, 1));
            Assert.Equal(0, map.HazardAt(1, 1));
            Assert.Equal(0f, map.SlopeAtNode(1, 1));
        }

        [Fact]
        public void TerrainMap_CopiesAreIndependent()
        {
            var map = Flat();
            map.CopyHazard()[0] = 2;
            map.CopyHeights()[0] = 99;
            map.CopySlopes()[0] = 45;
            Assert.Equal(0, map.HazardAt(0, 0));
            Assert.Equal(0f, map.HeightAt(0, 0));
            Assert.Equal(0f, map.SlopeAtNode(0, 0));
        }

        [Fact]
        public void TerrainMap_NodeAccessorsRejectBadNodes()
        {
            var map = Flat();
            Assert.Throws<ArgumentOutOfRangeException>(() => map.HeightAt(-1, 0));
            Assert.Throws<ArgumentOutOfRangeException>(() => map.HazardAt(0, Nodes));
            Assert.Throws<ArgumentOutOfRangeException>(() => map.SlopeAtNode(Nodes, 0));
        }

        [Fact]
        public void TerrainMeta_IsFrozen_AndTheCallersCopyStaysEditable()
        {
            var meta = new TerrainMetadata { Width = 3, Height = 3, SideM = 4 };
            var map = new TerrainMap(meta, new float[9], new byte[9]);
            Assert.True(map.Meta.IsFrozen);
            Assert.Throws<InvalidOperationException>(() => map.Meta.SideM = 400);
            meta.SideM = 400;                                   // the caller's object is untouched and editable
            Assert.Equal(2.0, map.Cell);
            Assert.Equal(4.0, map.Meta.SideM);
            var clone = map.Meta.Clone();
            clone.SideM = 8;
            Assert.False(clone.IsFrozen);
        }

        [Theory]
        [InlineData(0)]
        [InlineData(-4)]
        [InlineData(double.NaN)]
        [InlineData(double.PositiveInfinity)]
        public void TerrainMap_RejectsABadCellSize(double side)
        {
            var meta = new TerrainMetadata { Width = 3, Height = 3, SideM = side };
            Assert.Throws<ArgumentException>(() => new TerrainMap(meta, new float[9], new byte[9]));
        }

        [Fact]
        public void TerrainMap_NaNPointMapsToAFixedNode()
        {
            var map = Flat();
            map.NearestNode(new Point2(double.NaN, double.NaN), out int c, out int r);
            Assert.True(map.InBounds(c, r));
            Assert.True(double.IsFinite(map.ElevationAt(new Point2(double.NaN, 5))));
            Assert.False(map.IsInside(new Point2(double.NaN, 5)));
        }

        [Fact]
        public void EngineMission_PlacesCannotBeChanged()
        {
            var engine = NewEngine();
            var p1 = engine.Mission.Find("P1");
            Assert.True(p1.IsFrozen);
            Assert.Throws<InvalidOperationException>(() => p1.Position = new Point2(0, 0));
            Assert.Throws<InvalidOperationException>(() => p1.Id = "X");
            Assert.Throws<InvalidOperationException>(() => engine.Base.Type = "science");
        }

        [Fact]
        public void CallerPlaces_AreCopied_NotFrozenOrChanged()
        {
            var places = new[] { Wp("B", "base", 10, 10), Wp("P1", "science", 100, 10) };
            var mission = Custom(places, "B", "P1");
            Assert.False(places[1].IsFrozen);
            Assert.False(places[1].IsObjective);              // the mission no longer writes into the caller's objects
            places[1].Position = new Point2(50, 50);         // editing the original afterwards...
            Assert.Equal(100, mission.Find("P1").Position.X); // ...does not reach the mission
            var engine = NewEngine(mission: mission);
            engine.Update(At(BasePos));
            Assert.Equal(100, engine.Update(At(BasePos)).NextWaypoint.X);
        }

        [Fact]
        public void HazardZones_AreFrozenCopies()
        {
            var zone = new HazardZone { Type = "slope", MaxSlopeDeg = 30, BoxMin = new Point2(40, 40), BoxMax = new Point2(50, 50) };
            var mission = new MissionDefinition("m", "o", new[] { Wp("B", "base", 10, 10) }, new[] { "B" }, new[] { zone });
            zone.MaxSlopeDeg = 5;
            Assert.Equal("high", mission.HazardZones[0].Severity);
            Assert.Throws<InvalidOperationException>(() => mission.HazardZones[0].MaxSlopeDeg = 1);
        }

        [Fact]
        public void MissionLists_AreReadOnly()
        {
            var mission = Square();
            Assert.Throws<NotSupportedException>(() => ((IList<Waypoint>)mission.Waypoints).Add(Wp("X", "science", 1, 1)));
            Assert.Throws<NotSupportedException>(() => ((IList<string>)mission.ObjectiveIds).Clear());
        }

        [Fact]
        public void Options_ChangedAfterCreation_HaveNoEffect()
        {
            var options = new NavigationOptions();
            var engine = NewEngine(options: options);
            options.ReachRadiusM = 1000;                       // would complete every stop at once
            engine.Update(At(BasePos));
            var resp = engine.Update(At(BasePos));
            Assert.Equal("active", resp.Objectives[1].Status);
        }

        [Fact]
        public void RouteOptions_ChangedAfterCreation_HaveNoEffect()
        {
            var rough = Helpers.TestTerrain.WithHazard(Flat(), (c, r) => 3);
            var ro = new RouteOptions();
            var engine = NavigationEngineFactory.Create(rough, Square(), routeOptions: ro);
            double before = engine.PlanRoute(BasePos, P1Pos).EstimatedMinutes;
            ro.RoughFactor = 10;
            Assert.Equal(before, engine.PlanRoute(BasePos, P1Pos).EstimatedMinutes, 9);
        }

        [Theory]
        [InlineData("ReachRadiusM", -1.0)]
        [InlineData("OffRouteM", double.NaN)]
        [InlineData("LookAheadM", double.PositiveInfinity)]
        [InlineData("PolylineToleranceM", -0.5)]
        [InlineData("MaxSlopeDeg", 0.0)]
        [InlineData("MaxSlopeDeg", 95.0)]
        public void InvalidNavigationOptions_AreRejected(string name, double value)
        {
            var options = new NavigationOptions();
            typeof(NavigationOptions).GetProperty(name).SetValue(options, value);
            Assert.Throws<ArgumentOutOfRangeException>(() => NavigationEngineFactory.Create(Flat(), Square(), options));
        }

        [Theory]
        [InlineData("MaxSlopeDeg", double.NaN)]
        [InlineData("RoughFactor", 0.0)]
        [InlineData("RoughFactor", -1.0)]
        [InlineData("HeuristicWeight", -1.0)]
        [InlineData("HeuristicWeight", double.PositiveInfinity)]
        public void InvalidRouteOptions_AreRejected(string name, double value)
        {
            var ro = new RouteOptions();
            typeof(RouteOptions).GetProperty(name).SetValue(ro, value);
            Assert.Throws<ArgumentOutOfRangeException>(() => NavigationEngineFactory.Create(Flat(), Square(), routeOptions: ro));
        }
    }

    [Trait("Method", "MissionDefinition")]
    public class MissionValidationTests
    {
        [Fact]
        public void DuplicatePlaceIds_AreRejected()
        {
            var ex = Assert.Throws<ArgumentException>(() => Custom(new[] { Wp("B", "base", 10, 10), Wp("B", "science", 20, 20) }, "B"));
            Assert.Contains("twice", ex.Message);
        }

        [Fact]
        public void NullPlace_IsRejected()
        {
            Assert.Throws<ArgumentException>(() => Custom(new[] { Wp("B", "base", 10, 10), null }, "B"));
        }

        [Theory]
        [InlineData(null)]
        [InlineData("")]
        public void PlaceWithoutId_IsRejected(string id)
        {
            Assert.Throws<ArgumentException>(() => Custom(new[] { Wp(id, "base", 10, 10) }));
        }

        [Theory]
        [InlineData(double.NaN, 10)]
        [InlineData(10, double.PositiveInfinity)]
        public void PlaceWithNonFinitePosition_IsRejected(double x, double z)
        {
            Assert.Throws<ArgumentException>(() => Custom(new[] { Wp("B", "base", x, z) }, "B"));
        }

        [Fact]
        public void NullStepId_IsRejected()
        {
            Assert.Throws<ArgumentException>(() => Custom(new[] { Wp("B", "base", 10, 10) }, "B", null));
        }

        [Fact]
        public void NullZone_IsRejected()
        {
            Assert.Throws<ArgumentException>(() =>
                new MissionDefinition("m", "o", new[] { Wp("B", "base", 10, 10) }, new[] { "B" }, new HazardZone[] { null }));
        }

        [Fact]
        public void Find_IsExactAndNullSafe()
        {
            var mission = Square();
            Assert.Null(mission.Find(null));
            Assert.Null(mission.Find("p1"));                 // ids are case-sensitive
            Assert.Equal("P1", mission.Find("P1").Id);
            Assert.True(mission.HasObjective("P3"));
            Assert.False(mission.HasObjective("A1"));        // an antenna is a place, not a step
            Assert.False(mission.HasObjective(null));
        }

        [Fact]
        public void IsObjective_IsSetByTheMission()
        {
            var mission = Square();
            Assert.True(mission.Find("P2").IsObjective);
            Assert.False(mission.Find("A1").IsObjective);
            Assert.Equal(new[] { "A1", "B" }, mission.Antennas.Select(a => a.Id).OrderBy(i => i).ToArray());
            Assert.Equal(mission.ObjectiveIds, mission.Objectives.Select(o => o.Id).ToList());
        }
    }
}
