using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Marps.Engine.Geometry;
using Marps.Engine.Mission;
using Marps.Engine.Routing;
using Marps.Engine.Terrain;

namespace Marps.Engine.Navigation
{
    /// <summary>
    /// Navigation engine. No frontend dependencies: takes a <see cref="NavigationRequest"/>
    /// and returns a <see cref="NavigationResponse"/>. Keeps one session per astronaut
    /// (completed objectives and cached route). One instance serves calls one at a time (lock).
    /// </summary>
    public sealed class NavigationEngine
    {
        private readonly TerrainMap _map;
        private readonly MissionDefinition _mission;
        private readonly NavigationOptions _options;
        private readonly AStarRouter _router;
        private readonly Dictionary<string, SessionState> _sessions = new Dictionary<string, SessionState>();
        private readonly object _lock = new object();
        private long _useCounter;

        public NavigationEngine(TerrainMap map, MissionDefinition mission,
            NavigationOptions options = null, WalkingModel model = null, RouteOptions routeOptions = null)
        {
            _map = map ?? throw new ArgumentNullException(nameof(map));
            _mission = mission ?? throw new ArgumentNullException(nameof(mission));
            _options = options ?? new NavigationOptions();
            if (routeOptions == null) routeOptions = new RouteOptions { MaxSlopeDeg = _options.MaxSlopeDeg };
            _router = new AStarRouter(map, model, routeOptions);
        }

        public TerrainMap Map => _map;
        public MissionDefinition Mission => _mission;

        /// <summary>Direct route between two points, without a session (for planning from tools).</summary>
        public RouteResult PlanRoute(Point2 from, Point2 to)
        {
            lock (_lock) return _router.Find(from, to);
        }

        public NavigationResponse Update(NavigationRequest req)
        {
            if (req == null) throw new ArgumentNullException(nameof(req));
            var clock = Stopwatch.StartNew();

            lock (_lock)
            {
                string sid = string.IsNullOrEmpty(req.SessionId) ? "default" : req.SessionId;
                if (!_sessions.TryGetValue(sid, out var session))
                {
                    if (_sessions.Count >= Math.Max(1, _options.MaxSessions)) EvictLeastRecentlyUsed();
                    session = new SessionState();
                    _sessions[sid] = session;
                }
                session.LastUsed = ++_useCounter;

                var pos = new Point2(req.X, req.Z);
                var response = new NavigationResponse
                {
                    SessionId = sid,
                    MissionTimeS = req.MissionTimeS,
                    Position = new PositionInfo
                    {
                        X = req.X,
                        Z = req.Z,
                        Inside = _map.IsInside(pos),
                        ElevationM = _map.ElevationAt(pos),
                    },
                };

                var steps = _mission.ObjectiveIds;

                // 1. Progress by proximity: the current step is completed if the path travelled since the last
                //    sample passes within ReachRadiusM of its point (so a slow sampling rate cannot skip it).
                // Only the first pending step is checked against the travelled path; later steps are checked by
                // proximity to the current position (otherwise the departure base would be credited again on return).
                Point2 previous = session.HasLast ? session.LastPos : pos;
                bool firstStep = true;
                while (session.Step < steps.Count)
                {
                    var stepPos = _mission.Find(steps[session.Step]).Position;
                    double d = firstStep
                        ? PathSimplifier.DistanceToSegment(stepPos, previous, pos)
                        : pos.DistanceTo(stepPos);
                    if (d > _options.ReachRadiusM) break;
                    session.Step++;
                    firstStep = false;
                }
                session.LastPos = pos;
                session.HasLast = true;

                // 2. The astronaut can jump to a later step with that id (e.g. return to base "B").
                if (!string.IsNullOrEmpty(req.ActiveWaypointId))
                {
                    for (int k = session.Step; k < steps.Count; k++)
                    {
                        if (steps[k] == req.ActiveWaypointId)
                        {
                            session.Step = k;
                            break;
                        }
                    }
                }

                Waypoint target = session.Step < steps.Count ? _mission.Find(steps[session.Step]) : null;

                // 3. Route: recalculated if the target changed, if requested, or if the astronaut moved away from the route.
                bool replanned = false;
                if (target != null)
                {
                    bool needPlan = session.Route == null || session.TargetId != target.Id || req.ForceReplan;
                    if (!needPlan)
                    {
                        session.Nearest(pos, out _, out double offDist);
                        needPlan = offDist > _options.OffRouteM;
                    }
                    if (needPlan)
                    {
                        session.Route = _router.Find(pos, target.Position);
                        session.TargetId = target.Id;
                        session.PrepareCumulative();
                        replanned = true;
                    }

                    response.Route = BuildRouteInfo(session, target, pos, replanned);
                    response.NextWaypoint = BuildNextWaypoint(target, pos, req.HeadingDeg);
                }
                else
                {
                    response.MissionComplete = true;
                    session.Route = null;
                    session.TargetId = null;
                    session.PrepareCumulative();
                }

                for (int k = 0; k < steps.Count; k++)
                {
                    var w = _mission.Find(steps[k]);
                    response.Objectives.Add(new ObjectiveInfo
                    {
                        Step = k,
                        Id = w.Id,
                        Name = w.Name,
                        X = w.Position.X,
                        Z = w.Position.Z,
                        Status = k < session.Step ? "done" : (k == session.Step ? "active" : "pending"),
                    });
                }

                response.Hazards = NearbyHazards(pos);
                response.Layers = BuildLayers(pos, out int visible);
                response.Layers.VisibleAntennas = visible;
                response.Alerts = BuildAlerts(req, pos, response, target != null);

                clock.Stop();
                response.ComputeMs = clock.Elapsed.TotalMilliseconds;
                return response;
            }
        }

        private RouteInfo BuildRouteInfo(SessionState s, Waypoint target, Point2 pos, bool replanned)
        {
            var info = new RouteInfo
            {
                Target = target.Id,
                Replanned = replanned,
                Found = s.Route != null && s.Route.Found,
            };
            if (!info.Found) return info;

            var route = s.Route;
            info.LengthM = route.HorizontalLengthM;
            info.EstimatedMinutes = route.EstimatedMinutes;
            info.AscentM = route.AscentM;
            info.DescentM = route.DescentM;
            info.MaxSlopeDeg = route.MaxEdgeSlopeDeg;

            int i = s.Nearest(pos, out _, out double d);
            info.RemainingM = (s.Cumulative[s.Cumulative.Count - 1] - s.Cumulative[i]) + d;

            var tail = new List<Point2> { pos };
            for (int k = i; k < route.Points.Count; k++) tail.Add(route.Points[k]);
            info.Points = PathSimplifier.Simplify(tail, _options.PolylineToleranceM);
            return info;
        }

        private NextWaypointInfo BuildNextWaypoint(Waypoint target, Point2 pos, double headingDeg)
        {
            double bearing = Bearing.Between(pos, target.Position);
            return new NextWaypointInfo
            {
                Id = target.Id,
                Type = target.Type,
                Name = target.Name,
                X = target.Position.X,
                Z = target.Position.Z,
                DistanceM = pos.DistanceTo(target.Position),
                BearingDeg = bearing,
                RelativeBearingDeg = Bearing.Relative(bearing, headingDeg),
            };
        }

        private List<HazardInfo> NearbyHazards(Point2 pos)
        {
            return _mission.HazardZones
                .Select(z => new { Zone = z, Dist = z.DistanceTo(pos) })
                .Where(x => x.Dist <= _options.HazardRadiusM)
                .OrderBy(x => x.Dist)
                .Take(10)
                .Select(x => new HazardInfo
                {
                    Type = x.Zone.Type,
                    Severity = x.Zone.Severity,
                    DistanceM = x.Dist,
                    CenterX = x.Zone.Center.X,
                    CenterZ = x.Zone.Center.Z,
                    BoxMinX = x.Zone.BoxMin.X,
                    BoxMinZ = x.Zone.BoxMin.Z,
                    BoxMaxX = x.Zone.BoxMax.X,
                    BoxMaxZ = x.Zone.BoxMax.Z,
                    MaxSlopeDeg = x.Zone.MaxSlopeDeg,
                    AreaM2 = x.Zone.AreaM2,
                }).ToList();
        }

        private LayerInfo BuildLayers(Point2 pos, out int visibleCount)
        {
            _map.NearestNode(pos, out int c, out int r);
            var layers = new LayerInfo
            {
                SlopeDeg = _map.SlopeDeg[_map.Index(c, r)],
                HazardClass = _map.Hazard[_map.Index(c, r)],
                ElevationM = _map.ElevationAt(pos),
            };

            visibleCount = 0;
            foreach (var ant in _mission.Antennas)
            {
                bool visible = LineOfSight.Clear(_map, pos, _options.EyeHeightM, ant.Position, _options.AntennaHeightM);
                if (visible) visibleCount++;
                layers.Antennas.Add(new AntennaInfo { Id = ant.Id, Visible = visible });
            }
            return layers;
        }

        private List<AlertInfo> BuildAlerts(NavigationRequest req, Point2 pos, NavigationResponse resp, bool hasTarget)
        {
            var alerts = new List<AlertInfo>();

            if (!resp.Position.Inside)
                alerts.Add(new AlertInfo { Code = "OFF_MAP", Severity = "warning", Message = "Outside the map area." });

            // Terrain ahead, following the look direction.
            double rad = req.HeadingDeg * Math.PI / 180.0;
            double ex = Math.Sin(rad), nz = Math.Cos(rad);
            double worstSlope = 0;
            int worstHazard = 0;
            for (double d = 0; d <= _options.LookAheadM; d += _map.Cell)
            {
                var p = new Point2(pos.X + ex * d, pos.Z + nz * d);
                if (!_map.IsInside(p)) break;
                _map.NearestNode(p, out int c, out int r);
                int idx = _map.Index(c, r);
                worstSlope = Math.Max(worstSlope, _map.SlopeDeg[idx]);
                worstHazard = Math.Max(worstHazard, _map.Hazard[idx]);
            }
            if (worstHazard == 2)
                alerts.Add(new AlertInfo
                {
                    Code = "HAZARD_AHEAD",
                    Severity = "danger",
                    Message = $"Slope up to {worstSlope:0}° in the next {_options.LookAheadM:0} m.",
                });
            else if (worstHazard == 3)
                alerts.Add(new AlertInfo
                {
                    Code = "HAZARD_AHEAD",
                    Severity = "warning",
                    Message = "Zona rugosa adelante.",
                });
            else if (worstSlope >= _options.WarnSlopeDeg)
                alerts.Add(new AlertInfo
                {
                    Code = "HAZARD_AHEAD",
                    Severity = "warning",
                    Message = $"Slope up to {worstSlope:0}° ahead.",
                });

            // Signal: how many masts/base have line of sight.
            int antennas = resp.Layers?.Antennas.Count ?? 0;
            int visible = resp.Layers?.VisibleAntennas ?? 0;
            if (antennas > 0 && visible == 0)
                alerts.Add(new AlertInfo { Code = "SIGNAL_LOSS", Severity = "warning", Message = "No line of sight to any mast." });
            else if (antennas > 0 && visible == 1)
                alerts.Add(new AlertInfo { Code = "SIGNAL_LIMITED", Severity = "info", Message = "Only 1 mast with line of sight." });

            if (req.SpaceWeather != null && req.SpaceWeather.StormActive)
                alerts.Add(new AlertInfo
                {
                    Code = "SOLAR_STORM",
                    Severity = "warning",
                    Message = $"Solar storm ({req.SpaceWeather.Level ?? "no level"}). Seek shelter.",
                });

            if (hasTarget && resp.Route != null && !resp.Route.Found)
                alerts.Add(new AlertInfo { Code = "NO_ROUTE", Severity = "danger", Message = "No safe route to the target." });

            return alerts;
        }

        /// <summary>Drops the session that was used longest ago. Called with the lock held.</summary>
        private void EvictLeastRecentlyUsed()
        {
            string oldestId = null;
            long oldest = long.MaxValue;
            foreach (var pair in _sessions)
            {
                if (pair.Value.LastUsed < oldest)
                {
                    oldest = pair.Value.LastUsed;
                    oldestId = pair.Key;
                }
            }
            if (oldestId != null) _sessions.Remove(oldestId);
        }

        private sealed class SessionState
        {
            /// <summary>Value of the engine's use counter when this session was last used.</summary>
            public long LastUsed { get; set; }

            /// <summary>Index of the current step. Previous steps are completed.</summary>
            public int Step { get; set; }
            public Point2 LastPos { get; set; }
            public bool HasLast { get; set; }
            public string TargetId { get; set; }
            public RouteResult Route { get; set; }
            public List<double> Cumulative { get; } = new List<double>();

            public void PrepareCumulative()
            {
                Cumulative.Clear();
                if (Route == null || !Route.Found) return;
                double acc = 0;
                for (int i = 0; i < Route.Points.Count; i++)
                {
                    if (i > 0) acc += Route.Points[i - 1].DistanceTo(Route.Points[i]);
                    Cumulative.Add(acc);
                }
            }

            /// <summary>Index of the route node closest to p, and the distance to it.</summary>
            public int Nearest(Point2 p, out Point2 nearestPoint, out double distance)
            {
                nearestPoint = default(Point2);
                distance = double.MaxValue;
                if (Route == null || !Route.Found || Route.Points.Count == 0) return 0;
                int best = 0;
                for (int i = 0; i < Route.Points.Count; i++)
                {
                    double d = p.DistanceTo(Route.Points[i]);
                    if (d < distance)
                    {
                        distance = d;
                        best = i;
                    }
                }
                nearestPoint = Route.Points[best];
                return best;
            }

            // Overload for calls that do not use the point
            public int Nearest(Point2 p, out double distance) => Nearest(p, out _, out distance);
        }
    }
}
