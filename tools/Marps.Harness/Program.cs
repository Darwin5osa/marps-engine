using System;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Http;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text.Json;
using Marps.Engine.Abstractions;
using Marps.Engine.Geometry;
using Marps.Engine.Json;
using Marps.Engine.Mission;
using Marps.Engine.Navigation;
using Marps.Engine.Routing;
using Marps.Engine.Terrain;

// Manual test harness: serves the engine over HTTP and a web page (wwwroot/index.html).
// The engine only sees interfaces: the test "helmet" is WebSensor (IAstronautSensor) and the output is LastHudSink (IHudSink).
// In production, Unity or the hardware implement the same interfaces.

var dataDir = ResolveDataDir();
var processed = Path.Combine(dataDir, "processed");
if (!File.Exists(Path.Combine(processed, "terrain.json")))
{
    Console.Error.WriteLine($"No encuentro {processed}\\terrain.json. Ejecutar: dotnet run --project tools/Marps.DataImport -- context");
    return 1;
}

ITerrainProvider terrainProvider = new FileTerrainProvider(processed);
TerrainMap terrain = terrainProvider.Load();
MissionDefinition mission = MissionLoader.Load(
    Path.Combine(dataDir, "mission.json"), Path.Combine(dataDir, "hazards.json"), terrain.Meta);
var engine = new NavigationEngine(terrain, mission);

var sensor = new WebSensor();
var sink = new LastHudSink();
var service = new NavigationService(engine, sensor, sink, null);

// Terrain layer for the page: hazard and slope per node (1 byte each, base64).
var slopeBytes = new byte[terrain.SlopeDeg.Length];
for (int i = 0; i < slopeBytes.Length; i++)
    slopeBytes[i] = (byte)Math.Min(255, Math.Round(terrain.SlopeDeg[i]));
string hazardB64 = Convert.ToBase64String(terrain.Hazard);
string slopeB64 = Convert.ToBase64String(slopeBytes);

var referenceLegs = LoadReferenceLegs(Path.Combine(dataDir, "reference_routes.json"));

var builder = WebApplication.CreateBuilder(args);
builder.WebHost.UseUrls(Environment.GetEnvironmentVariable("MARPS_URL") ?? "http://localhost:5080");
var app = builder.Build();

app.UseDefaultFiles();
app.UseStaticFiles();

app.MapGet("/api/health", () => Results.Json(new { ok = true, map = terrain.Width, cell = terrain.Cell }, MarpsJson.Options));

app.MapGet("/api/mission", () =>
{
    var meta = terrain.Meta;
    return Results.Json(new
    {
        name = mission.Name,
        objective = mission.Objective,
        width = terrain.Width,
        height = terrain.Height,
        cell = terrain.Cell,
        side = meta.SideM,
        originX = meta.OriginX,
        originY = meta.OriginY,
        zMin = meta.ZMin,
        zMax = meta.ZMax,
        hazardB64,
        slopeB64,
        objectiveOrder = mission.ObjectiveIds,
        waypoints = mission.Waypoints.Select(w => new
        {
            id = w.Id, type = w.Type, name = w.Name, reason = w.Reason,
            x = w.Position.X, z = w.Position.Z, isObjective = w.IsObjective,
            projX = w.ProjectionX, projY = w.ProjectionY,
        }),
        zones = mission.HazardZones.Select(z => new
        {
            type = z.Type, areaM2 = z.AreaM2, maxSlopeDeg = z.MaxSlopeDeg, severity = z.Severity,
            centerX = z.Center.X, centerZ = z.Center.Z,
            boxMinX = z.BoxMin.X, boxMinZ = z.BoxMin.Z, boxMaxX = z.BoxMax.X, boxMaxZ = z.BoxMax.Z,
        }),
        referenceLegs,
    }, MarpsJson.Options);
});

// Direct route between two local points (x east, z north).
app.MapPost("/api/route", (RouteQuery q) =>
{
    var clock = Stopwatch.StartNew();
    RouteResult r = engine.PlanRoute(new Point2(q.FromX, q.FromZ), new Point2(q.ToX, q.ToZ));
    clock.Stop();
    return Results.Json(new
    {
        found = r.Found,
        lengthM = r.HorizontalLengthM,
        minutes = r.EstimatedMinutes,
        ascentM = r.AscentM,
        descentM = r.DescentM,
        maxSlopeDeg = r.MaxEdgeSlopeDeg,
        expansions = r.Expansions,
        computeMs = r.ComputeMs,
        totalMs = clock.Elapsed.TotalMilliseconds,
        points = PathSimplifier.Simplify(r.Points, 0.5),
        nodes = r.Points.Count,
    }, MarpsJson.Options);
});

// Astronaut state "sent" by the test helmet.
app.MapPost("/api/sensor", (NavigationRequest req) =>
{
    sensor.Set(req);
    return Results.Json(new { ok = true }, MarpsJson.Options);
});

// Runs the full service cycle: sensor -> engine -> sink, and returns what the HUD would show.
app.MapGet("/api/hud", () =>
{
    var clock = Stopwatch.StartNew();
    NavigationResponse resp = service.Step();
    clock.Stop();
    return Results.Json(new { hud = resp, serverMs = clock.Elapsed.TotalMilliseconds }, MarpsJson.Options);
});

// Shortcut to test the engine without going through the sensor.
app.MapPost("/api/navigate", (NavigationRequest req) =>
{
    var clock = Stopwatch.StartNew();
    NavigationResponse resp = engine.Update(req);
    clock.Stop();
    return Results.Json(new { hud = resp, serverMs = clock.Elapsed.TotalMilliseconds }, MarpsJson.Options);
});

app.Run();
return 0;

// ---------------------------------------------------------------------------

static string ResolveDataDir()
{
    string env = Environment.GetEnvironmentVariable("MARPS_DATA");
    if (!string.IsNullOrEmpty(env)) return env;

    var dir = new DirectoryInfo(AppContext.BaseDirectory);
    while (dir != null)
    {
        string candidate = Path.Combine(dir.FullName, "context");
        if (Directory.Exists(Path.Combine(candidate, "processed"))) return candidate;
        dir = dir.Parent;
    }
    return Path.Combine(Directory.GetCurrentDirectory(), "context");
}

static List<object> LoadReferenceLegs(string referencePath)
{
    var list = new List<object>();
    if (!File.Exists(referencePath)) return list;

    using var doc = JsonDocument.Parse(File.ReadAllText(referencePath));
    foreach (var leg in doc.RootElement.GetProperty("with_rule").EnumerateArray())
    {
        list.Add(new
        {
            leg = leg.GetProperty("leg").GetString(),
            lengthM = leg.GetProperty("length_m").GetDouble(),
            minutes = leg.GetProperty("minutes").GetDouble(),
        });
    }
    return list;
}

public sealed class RouteQuery
{
    public double FromX { get; set; }
    public double FromZ { get; set; }
    public double ToX { get; set; }
    public double ToZ { get; set; }
}

/// <summary>Test sensor: the state is set by the web page (POST /api/sensor).</summary>
public sealed class WebSensor : IAstronautSensor
{
    private readonly object _lock = new object();
    private NavigationRequest _state = new NavigationRequest { SessionId = "web", X = 300, Z = 1700 };

    public void Set(NavigationRequest state)
    {
        lock (_lock) _state = state;
    }

    public NavigationRequest Read()
    {
        lock (_lock)
        {
            // Copy so the engine can annotate fields without touching the stored state.
            return new NavigationRequest
            {
                SessionId = _state.SessionId,
                X = _state.X,
                Z = _state.Z,
                HeadingDeg = _state.HeadingDeg,
                PitchDeg = _state.PitchDeg,
                MissionTimeS = _state.MissionTimeS,
                ActiveWaypointId = _state.ActiveWaypointId,
                ForceReplan = _state.ForceReplan,
            };
        }
    }
}

/// <summary>Last published HUD. Used for inspection from the page.</summary>
public sealed class LastHudSink : IHudSink
{
    private readonly object _lock = new object();
    public NavigationResponse Last { get; private set; }

    public void Publish(NavigationResponse response)
    {
        lock (_lock) Last = response;
    }
}
