using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using Marps.Engine.Geometry;
using Marps.Engine.Mission;
using Marps.Engine.Navigation;
using Marps.Engine.Terrain;

// Simulates how the engine behaves inside a Meta Quest 2 application.
//
// There is no Quest 2 emulator or XR2 CPU model available for .NET, so this is an approximation:
//   * the process is pinned to one logical core (the headset app runs its engine code on a single thread);
//   * timings are multiplied by --cpu-factor (default 1.0 = this PC). Calibrate it on the headset if possible;
//   * each call's allocated bytes and GC collections are reported, because Unity's Mono GC pauses the frame;
//   * the frame budget is --budget-ms per engine call (default 2.0 ms, an assumption, not a Meta number).
//
// Usage: dotnet run -c Release --project tools/Marps.Quest2Sim -- [data_dir] [--cpu-factor 1.0] [--budget-ms 2.0] [--frames 20000] [--speed 1.4]

static string Arg(string[] args, string name, string def)
{
    for (int i = 0; i < args.Length - 1; i++)
        if (args[i] == name) return args[i + 1];
    return def;
}

string dataDir = args.FirstOrDefault(a => !a.StartsWith("--"))
                 ?? Environment.GetEnvironmentVariable("MARPS_DATA")
                 ?? "context";
dataDir = Path.GetFullPath(dataDir);
double cpuFactor = double.Parse(Arg(args, "--cpu-factor", "1.0"), System.Globalization.CultureInfo.InvariantCulture);
double budgetMs = double.Parse(Arg(args, "--budget-ms", "2.0"), System.Globalization.CultureInfo.InvariantCulture);
int frames = int.Parse(Arg(args, "--frames", "20000"));
double speed = double.Parse(Arg(args, "--speed", "1.4"), System.Globalization.CultureInfo.InvariantCulture);
const double FrameHz = 72.0;

try { Process.GetCurrentProcess().ProcessorAffinity = (IntPtr)1; } catch (Exception ex) { Console.WriteLine($"(affinity not set: {ex.Message})"); }

Console.WriteLine($"Quest 2 simulation | data: {dataDir}");
Console.WriteLine($"  single core, cpu-factor {cpuFactor}, engine budget {budgetMs} ms per call, walking {speed} m/s, frames {frames}");
Console.WriteLine($"  .NET {Environment.Version}, {Environment.OSVersion}, workstation GC, pinned to 1 logical core");

var terrain = TerrainFiles.Load(Path.Combine(dataDir, "processed"));
var mission = MissionLoader.Load(Path.Combine(dataDir, "mission.json"), Path.Combine(dataDir, "hazards.json"), terrain.Meta);
bool background = Arg(args, "--background", "false") == "true";
NavigationEngine Make() => new NavigationEngine(terrain, mission, new NavigationOptions { BackgroundReplanning = background });

// Path the astronaut walks: B -> P1 -> P2 -> P3 -> P4 -> B, following the planned routes.
var planner = new NavigationEngine(terrain, mission);
var path = new List<Point2>();
var order = mission.ObjectiveIds;
for (int i = 0; i < order.Count - 1; i++)
{
    var leg = planner.PlanRoute(mission.Find(order[i]).Position, mission.Find(order[i + 1]).Position);
    foreach (var p in leg.Points)
        if (path.Count == 0 || path[path.Count - 1].DistanceTo(p) > 1e-9) path.Add(p);
}
var cumulative = new double[path.Count];
for (int i = 1; i < path.Count; i++) cumulative[i] = cumulative[i - 1] + path[i - 1].DistanceTo(path[i]);
double pathLength = cumulative[cumulative.Length - 1];
Console.WriteLine($"  walking path: {path.Count} points, {pathLength:0} m (about {pathLength / speed / 60:0.0} min at {speed} m/s)");

// Position along the path after walking 'distance' metres from the start (linear search, moving index).
Point2 PointAt(double distance, out double headingDeg)
{
    distance = Math.Min(distance, pathLength - 1e-6);
    int i = 0;
    while (i < cumulative.Length - 2 && cumulative[i + 1] < distance) i++;
    var a = path[i];
    var b = path[i + 1];
    double seg = cumulative[i + 1] - cumulative[i];
    double t = seg > 0 ? (distance - cumulative[i]) / seg : 0;
    headingDeg = Bearing.Normalize360(Math.Atan2(b.X - a.X, b.Z - a.Z) * 180.0 / Math.PI);
    return new Point2(a.X + (b.X - a.X) * t, a.Z + (b.Z - a.Z) * t);
}

var results = new List<Scenario>();

Scenario Run(string name, int calls, Func<int, NavigationRequest> requestAt, NavigationEngine engine, int warmup = 300)
{
    for (int i = 0; i < warmup; i++) engine.Update(requestAt(-1 - i));

    var times = new double[calls];
    var allocs = new long[calls];
    int replans = 0;
    int g0 = GC.CollectionCount(0), g1 = GC.CollectionCount(1), g2 = GC.CollectionCount(2);
    for (int i = 0; i < calls; i++)
    {
        var req = requestAt(i);
        long a0 = GC.GetAllocatedBytesForCurrentThread();
        var sw = Stopwatch.StartNew();
        var resp = engine.Update(req);
        double ms = sw.Elapsed.TotalMilliseconds * cpuFactor;
        long a1 = GC.GetAllocatedBytesForCurrentThread();
        times[i] = ms;
        allocs[i] = a1 - a0;
        if (resp.Route != null && resp.Route.Replanned) replans++;
    }
    var s = new Scenario
    {
        Name = name,
        Calls = calls,
        P50 = Pct(times, 0.50),
        P95 = Pct(times, 0.95),
        P99 = Pct(times, 0.99),
        Max = times.Max(),
        OverBudgetPct = 100.0 * times.Count(t => t > budgetMs) / calls,
        AvgAllocBytes = allocs.Average(),
        P95AllocBytes = Pct(allocs.Select(a => (double)a).ToArray(), 0.95),
        TotalAllocMB = allocs.Sum() / 1048576.0,
        Gen0 = GC.CollectionCount(0) - g0,
        Gen1 = GC.CollectionCount(1) - g1,
        Gen2 = GC.CollectionCount(2) - g2,
        Replans = replans,
    };
    results.Add(s);
    return s;
}

static double Pct(double[] xs, double p)
{
    var sorted = (double[])xs.Clone();
    Array.Sort(sorted);
    int idx = Math.Max(0, Math.Min(sorted.Length - 1, (int)Math.Ceiling(p * sorted.Length) - 1));
    return sorted[idx];
}

// Scenario 1: walking the mission at 72 Hz (frame rate of the headset).
var e1 = Make();
double step72 = speed / FrameHz;
Run("walk, 72 Hz", frames, i =>
{
    int k = i < 0 ? 0 : i;
    double dist = Math.Min(k * step72, pathLength - 1e-6);
    var p = PointAt(dist, out double h);
    return new NavigationRequest { SessionId = "walker", X = p.X, Z = p.Z, HeadingDeg = h, MissionTimeS = k / FrameHz };
}, e1);

// Scenario 2: same walk sampled at 5 Hz (the rate suggested by the contract).
var e2 = Make();
double step5 = speed / 5.0;
Run("walk, 5 Hz", 5000, i =>
{
    int k = i < 0 ? 0 : i;
    double dist = Math.Min(k * step5, pathLength - 1e-6);
    var p = PointAt(dist, out double h);
    return new NavigationRequest { SessionId = "walker", X = p.X, Z = p.Z, HeadingDeg = h };
}, e2);

// Scenario 3: standing still at the base, looking around, 72 Hz (the common HUD case).
var e3 = Make();
var baseP = mission.Find("B").Position;
Run("idle at base, 72 Hz", frames, i =>
{
    int k = i < 0 ? 0 : i;
    return new NavigationRequest { SessionId = "idle", X = baseP.X, Z = baseP.Z, HeadingDeg = (k * 2.0) % 360 };
}, e3);

// Scenario 4: worst case, the position jumps to a random spot every call (forces a replan each time).
var e4 = Make();
var rnd = new Random(7);
Run("replan every call (random jumps)", 300, i =>
    new NavigationRequest { SessionId = "jumper", X = 100 + rnd.NextDouble() * 1800, Z = 100 + rnd.NextDouble() * 1800, HeadingDeg = rnd.Next(360) }, e4, warmup: 20);

// Report
var sb = new StringBuilder();
sb.AppendLine();
sb.AppendLine("| scenario | calls | p50 ms | p95 ms | p99 ms | max ms | over budget | replans | avg alloc / call | p95 alloc / call | total alloc | GC gen0/gen1/gen2 |");
sb.AppendLine("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|");
foreach (var s in results)
    sb.AppendLine($"| {s.Name} | {s.Calls} | {s.P50:0.000} | {s.P95:0.000} | {s.P99:0.000} | {s.Max:0.0} | {s.OverBudgetPct:0.0} % | {s.Replans} | {s.AvgAllocBytes / 1024:0.0} KB | {s.P95AllocBytes / 1024:0.0} KB | {s.TotalAllocMB:0.0} MB | {s.Gen0}/{s.Gen1}/{s.Gen2} |");
Console.Write(sb.ToString());
Console.WriteLine($"Peak working set: {Process.GetCurrentProcess().PeakWorkingSet64 / 1048576.0:0} MB, managed heap now: {GC.GetTotalMemory(false) / 1048576.0:0.0} MB");

if (Arg(args, "--out", null) is string outFile)
    File.WriteAllText(outFile, sb.ToString());

sealed class Scenario
{
    public string Name;
    public int Calls;
    public double P50, P95, P99, Max, OverBudgetPct;
    public double AvgAllocBytes, P95AllocBytes, TotalAllocMB;
    public int Gen0, Gen1, Gen2, Replans;
}
