using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using Marps.Engine.Abstractions;
using Marps.Engine.Geometry;
using Marps.Engine.Mission;
using Marps.Engine.Navigation;
using Marps.Engine.Terrain;
using Marps.Engine.Tests.Helpers;

namespace Marps.Engine.Tests.Interface
{
    /// <summary>
    /// Shared setup for the interface edge-case suite. Every test class in this folder carries
    /// [Trait("Method", "...")] with the INavigationEngine member (or frontend interface) it exercises,
    /// so results can be counted per method: dotnet test --filter Method=PlanTrip
    /// </summary>
    internal static class Fixtures
    {
        public const double Cell = 2.0;
        public const int Nodes = 60;                 // 118 m x 118 m

        public static readonly Point2 BasePos = new Point2(10, 10);
        public static readonly Point2 P1Pos = new Point2(100, 10);
        public static readonly Point2 P2Pos = new Point2(100, 100);
        public static readonly Point2 P3Pos = new Point2(10, 100);

        public static Waypoint Wp(string id, string type, double x, double z) =>
            new Waypoint { Id = id, Type = type, Name = id, Position = new Point2(x, z) };

        /// <summary>B -> P1 -> P2 -> P3 -> B on a flat square, plus antenna A1 (not a stop).</summary>
        public static MissionDefinition Square() => new MissionDefinition("square", "edge cases",
            new[]
            {
                Wp("B", "base", 10, 10), Wp("P1", "science", 100, 10), Wp("P2", "science", 100, 100),
                Wp("P3", "science", 10, 100), Wp("A1", "antenna", 60, 60),
            },
            new[] { "B", "P1", "P2", "P3", "B" }, new HazardZone[0]);

        public static MissionDefinition Custom(IEnumerable<Waypoint> places, params string[] order) =>
            new MissionDefinition("custom", "edge cases", places, order, new HazardZone[0]);

        public static TerrainMap Flat() => TestTerrain.Flat(Nodes, Cell);

        /// <summary>Flat map with a closed square ring of blocked terrain (hazard 2) around <paramref name="center"/>.</summary>
        public static TerrainMap Walled(Point2 center, double inner = 8, double outer = 10) =>
            TestTerrain.WithHazard(Flat(), (c, r) =>
            {
                double x = c * Cell, z = (Nodes - 1 - r) * Cell;
                double d = Math.Max(Math.Abs(x - center.X), Math.Abs(z - center.Z));
                return (byte)(d >= inner && d <= outer ? 2 : 0);
            });

        /// <summary>Flat map where only the node nearest to <paramref name="p"/> is blocked.</summary>
        public static TerrainMap BlockedNode(Point2 p) =>
            TestTerrain.WithHazard(Flat(), (c, r) =>
                (byte)(Math.Abs(c * Cell - p.X) < Cell / 2 && Math.Abs((Nodes - 1 - r) * Cell - p.Z) < Cell / 2 ? 2 : 0));

        public static INavigationEngine NewEngine(TerrainMap map = null, MissionDefinition mission = null, NavigationOptions options = null) =>
            new NavigationEngine(map ?? Flat(), mission ?? Square(), options);

        public static NavigationRequest At(double x, double z, string session = "s1", double heading = 0) =>
            new NavigationRequest { SessionId = session, X = x, Z = z, HeadingDeg = heading };

        public static NavigationRequest At(Point2 p, string session = "s1") => At(p.X, p.Z, session);

        public static string[] Ids(NavigationResponse r) => r.Objectives.Select(o => o.Id).ToArray();
        public static string[] Statuses(NavigationResponse r) => r.Objectives.Select(o => o.Status).ToArray();

        /// <summary>Leaves the base and reaches P1, so the active stop is P2.</summary>
        public static void ReachP1(INavigationEngine engine, string session = "s1")
        {
            engine.Update(At(BasePos, session));
            engine.Update(At(P1Pos, session));
        }

        /// <summary>Visits every stop and returns to base: the mission is complete.</summary>
        public static NavigationResponse CompleteMission(INavigationEngine engine, string session = "s1")
        {
            ReachP1(engine, session);
            engine.Update(At(P2Pos, session));
            engine.Update(At(P3Pos, session));
            return engine.Update(At(BasePos, session));
        }

        /// <summary>Repeats an update until the background search settles (or 5 s pass).</summary>
        public static NavigationResponse Settle(INavigationEngine engine, NavigationRequest req)
        {
            var resp = engine.Update(req);
            var clock = Stopwatch.StartNew();
            while (resp.Route != null && resp.Route.Pending && clock.ElapsedMilliseconds < 5000)
            {
                Thread.Sleep(5);
                resp = engine.Update(req);
            }
            return resp;
        }

        public static Point2 Last(List<Point2> points) => points[points.Count - 1];
    }
}
