using System;
using System.Collections.Generic;
using System.Linq;
using Marps.Engine.Geometry;
using Marps.Engine.Navigation;
using Marps.Engine.Routing;
using Marps.Engine.Tests.Helpers;
using Xunit;
using Xunit.Abstractions;

namespace Marps.Engine.Tests
{
    /// <summary>
    /// Tests using the real Deuteronilus Mensae terrain (context/processed).
    /// Skipped if the data was not generated with Marps.DataImport.
    /// </summary>
    public class RealDataTests
    {
        private readonly ITestOutputHelper _out;

        public RealDataTests(ITestOutputHelper output)
        {
            _out = output;
        }

        private static void RequireData() =>
            Skip.IfNot(RealData.Available, $"No processed data in {RealData.ProcessedDir}. Run Marps.DataImport.");

        [SkippableFact]
        public void Terrain_HasExpectedGridAndElevationRange()
        {
            RequireData();
            var t = RealData.Terrain;
            Assert.Equal(1025, t.Width);
            Assert.Equal(1025, t.Height);
            Assert.Equal(1.953125, t.Cell, 6);
            Assert.Equal(-3623.578, t.Heights.Min(), 2);
            Assert.Equal(-3481.388, t.Heights.Max(), 2);
        }

        [SkippableFact]
        public void Hazard_ProportionsMatchReport()
        {
            // Reference report: 88.7 % < 15°, 9,0 % entre 15° y 22°, 2,3 % > 22°
            RequireData();
            int n = RealData.Terrain.Hazard.Length;
            double c0 = RealData.Terrain.Hazard.Count(h => h == 0) / (double)n * 100;
            double c1 = RealData.Terrain.Hazard.Count(h => h == 1) / (double)n * 100;
            double c2 = RealData.Terrain.Hazard.Count(h => h == 2) / (double)n * 100;
            double c3 = RealData.Terrain.Hazard.Count(h => h == 3) / (double)n * 100;
            _out.WriteLine($"Hazard: 0={c0:0.0}% 1={c1:0.0}% 2={c2:0.0}% 3={c3:0.0}%");
            Assert.InRange(c1, 8.5, 9.5);
            Assert.InRange(c2, 2.0, 2.6);
            // The report counts the rough zone (3) within "under 15°": 74.9 + 13.8 = 88.7 %.
            Assert.InRange(c0 + c3, 87.5, 89.5);
        }

        [SkippableFact]
        public void SlopeMap_AgreesWithHazardClasses()
        {
            // The grid's hazard data must match the computed slope on most nodes.
            RequireData();
            var t = RealData.Terrain;
            int agree = 0, total = 0;
            for (int i = 0; i < t.Hazard.Length; i++)
            {
                if (t.Hazard[i] == 3) continue; // rough zone is not pure slope
                total++;
                int byAngle = t.SlopeDeg[i] > 22f ? 2 : (t.SlopeDeg[i] >= 15f ? 1 : 0);
                if (byAngle == t.Hazard[i]) agree++;
            }
            double ratio = agree / (double)total;
            _out.WriteLine($"Agreement between slope and data hazard: {ratio:P2}");
            Assert.True(ratio > 0.90, $"acuerdo {ratio:P2}");
        }

        [SkippableFact]
        public void Mission_LoadsAllPlacesAndZones()
        {
            RequireData();
            var m = RealData.Mission;
            Assert.Equal(new[] { "B", "P1", "P2", "P3", "P4", "B" }, m.ObjectiveIds);
            Assert.Equal(new Point2(300, 1700), m.Find("B").Position);
            Assert.Equal(new Point2(1450, 1400), m.Find("P2").Position);
            Assert.Equal(186, m.HazardZones.Count);
            var antennaIds = m.Antennas.Select(a => a.Id).ToList();
            Assert.Contains("B", antennaIds);
            Assert.Contains("A2", antennaIds);
            Assert.Contains("A3", antennaIds);
        }

        [SkippableFact]
        public void ReferenceLegs_AreComparedInLengthAndTime()
        {
            // The engine optimizes TIME (fitted model), not distance. So it may take a somewhat longer but faster path.
            // Criterion: time no worse than the reference (+2 %) and length within ±10 %.
            RequireData();
            var router = new AStarRouter(RealData.Terrain);

            foreach (var leg in ReferenceData.Legs)
            {
                var route = router.Find(leg.From, leg.To);
                Assert.True(route.Found, $"{leg.Name}: no route");

                double lenErr = (route.HorizontalLengthM - leg.LengthM) / leg.LengthM;
                double minErr = (route.EstimatedMinutes - leg.Minutes) / leg.Minutes;
                _out.WriteLine($"{leg.Name}: largo {route.HorizontalLengthM:0} m (ref {leg.LengthM}, {lenErr:+0.0%;-0.0%}), " +
                               $"{route.EstimatedMinutes:0.0} min (ref {leg.Minutes}, {minErr:+0.0%;-0.0%}), " +
                               $"max slope {route.MaxEdgeSlopeDeg:0.0}°, {route.ComputeMs:0} ms");

                Assert.True(Math.Abs(lenErr) <= 0.10, $"{leg.Name}: length error {lenErr:P1}");
                Assert.True(minErr <= 0.02, $"{leg.Name}: slower than the reference {minErr:P1}");
            }
        }

        [SkippableFact]
        public void ReferencePaths_CostUnderSameModel_IsNotBeatenByOurRoute()
        {
            // Cost of the reference route (reference_routes.json, with_rule) evaluated with OUR terrain and model.
            // Since A* is optimal over the grid, our route must not cost more than the reference trip
            // (except for a 1 % discretization margin: reference points are not on our nodes).
            RequireData();
            string referencePath = System.IO.Path.Combine(RealData.ContextDir, "reference_routes.json");
            Skip.IfNot(System.IO.File.Exists(referencePath), "reference_routes.json is missing");

            var doc = System.Text.Json.JsonDocument.Parse(System.IO.File.ReadAllText(referencePath));
            var legs = doc.RootElement.GetProperty("with_rule").EnumerateArray().ToList();
            var meta = RealData.Terrain.Meta;
            var model = WalkingModel.Fitted;
            var router = new AStarRouter(RealData.Terrain);

            for (int li = 0; li < legs.Count; li++)
            {
                var pts = legs[li].GetProperty("points").EnumerateArray()
                    .Select(p => meta.ProjectionToLocal(p[0].GetDouble(), p[1].GetDouble())).ToList();

                // Fine resampling (<= 0.5 m) of each leg: with points ~10 m apart the climbs would be averaged out.
                var terrain = RealData.Terrain;
                double refMinutes = 0;
                for (int i = 1; i < pts.Count; i++)
                {
                    double segLen = pts[i - 1].DistanceTo(pts[i]);
                    int steps = Math.Max(1, (int)Math.Ceiling(segLen / 0.5));
                    double stepLen = segLen / steps;
                    Point2 prev = pts[i - 1];
                    double prevH = terrain.ElevationAt(prev);
                    for (int k = 1; k <= steps; k++)
                    {
                        double t = k / (double)steps;
                        var p = new Point2(pts[i - 1].X + (pts[i].X - pts[i - 1].X) * t,
                                           pts[i - 1].Z + (pts[i].Z - pts[i - 1].Z) * t);
                        double h = terrain.ElevationAt(p);
                        refMinutes += model.EdgeMinutes(stepLen, h - prevH);
                        prevH = h;
                        prev = p;
                    }
                }

                var ours = router.Find(pts[0], pts[pts.Count - 1]);
                _out.WriteLine($"Leg {li + 1}: our route {ours.EstimatedMinutes:0.00} min vs reference evaluated {refMinutes:0.00} min");
                Assert.True(ours.Found);
                Assert.True(ours.EstimatedMinutes <= refMinutes * 1.01 + 1e-6,
                    $"leg {li + 1}: our route {ours.EstimatedMinutes:0.00} > reference {refMinutes:0.00}");
            }
        }

        [SkippableFact]
        public void FullMission_ChainedLegsTotalAboutFourPointSixKilometres()
        {
            RequireData();
            var router = new AStarRouter(RealData.Terrain);
            double total = 0, minutes = 0;
            foreach (var leg in ReferenceData.Legs)
            {
                var r = router.Find(leg.From, leg.To);
                Assert.True(r.Found);
                total += r.HorizontalLengthM;
                minutes += r.EstimatedMinutes;
            }
            _out.WriteLine($"Full mission: {total:0} m, {minutes:0.0} min (ref 4600 m, 87 min)");
            Assert.InRange(total, 4600 * 0.95, 4600 * 1.05);
            Assert.InRange(minutes, 87 * 0.9, 87 * 1.1);
        }

        [SkippableFact]
        public void NoRouteUsesStepOverTwentyTwoDegrees_IncludingSteepSpot()
        {
            // Report sabotage test: the reference planner stepped on a 23.7° point at X 400, Y 2569133.
            RequireData();
            var t = RealData.Terrain;
            var router = new AStarRouter(t);
            foreach (var leg in ReferenceData.Legs)
            {
                var route = router.Find(leg.From, leg.To);
                Assert.True(route.Found);
                Assert.True(route.MaxEdgeSlopeDeg <= 22.0 + 1e-6, $"{leg.Name} pisa {route.MaxEdgeSlopeDeg:0.0}°");
                Assert.Equal(0, route.Points.Count(p =>
                {
                    t.NearestNode(p, out int c, out int r);
                    return t.Hazard[t.Index(c, r)] == 2;
                }));
            }
        }

        [SkippableFact]
        public void SteepSpot_IsSteepInTheData()
        {
            RequireData();
            // The report's 23.7° point must really be steep in the data.
            double s = RealData.Terrain.SlopeAt(ReferenceData.SteepSpot);
            _out.WriteLine($"Slope at the report's point: {s:0.0}°");
            Assert.True(s > 18, $"slope {s}");
        }

        [SkippableFact]
        public void Engine_RespondsQuicklyOnRealMap()
        {
            RequireData();
            var engine = new NavigationEngine(RealData.Terrain, RealData.Mission);
            var b = ReferenceData.Legs[0].From;

            // First call (computes the route), then HUD calls with the cached route.
            var first = engine.Update(new NavigationRequest { SessionId = "perf", X = b.X, Z = b.Z, HeadingDeg = 0 });
            var times = new List<double>();
            for (int i = 0; i < 200; i++)
            {
                var r = engine.Update(new NavigationRequest { SessionId = "perf", X = b.X + i * 0.05, Z = b.Z, HeadingDeg = 0 });
                times.Add(r.ComputeMs);
            }
            times.Sort();
            double p95 = times[(int)(times.Count * 0.95)];
            _out.WriteLine($"First route: {first.ComputeMs:0.0} ms. HUD with cache: median {times[100]:0.000} ms, p95 {p95:0.000} ms");
            Assert.True(first.ComputeMs < 2000, $"first route {first.ComputeMs} ms");
            Assert.True(p95 < 20, $"p95 HUD {p95} ms");
        }

        [SkippableFact]
        public void Engine_ReplanInRealRoute_CompletesFirstLeg()
        {
            RequireData();
            var engine = new NavigationEngine(RealData.Terrain, RealData.Mission);
            var b = ReferenceData.Legs[0].From;
            var p1 = ReferenceData.Legs[0].To;
            var resp = engine.Update(new NavigationRequest { SessionId = "leg", X = b.X, Z = b.Z, HeadingDeg = 90 });
            Assert.True(resp.Route.Found);
            Assert.Equal("P1", resp.NextWaypoint.Id);
            Assert.Equal(p1.X, resp.NextWaypoint.X, 6);
            Assert.True(resp.Route.Points.Count > 2);
        }
    }
}
