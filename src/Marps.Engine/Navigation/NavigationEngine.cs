using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
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
    /// With <see cref="NavigationOptions.BackgroundReplanning"/> the route search runs on a worker thread
    /// and the instance should be disposed when it is no longer used.
    /// </summary>
    public sealed class NavigationEngine : IDisposable
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
            _base = FindBase(mission);
            if (_options.BackgroundReplanning) StartPlanWorker(model, routeOptions);
        }

        public TerrainMap Map => _map;
        public MissionDefinition Mission => _mission;

        /// <summary>The base used for emergency return, or null if the mission has none.</summary>
        public Waypoint Base => _base;

        private readonly Waypoint _base;

        /// <summary>
        /// The emergency base: the departure step if it is a base, else the final step if it is one,
        /// else any place of type "base".
        /// </summary>
        private static Waypoint FindBase(MissionDefinition mission)
        {
            var steps = mission.ObjectiveIds;
            if (steps.Count > 0)
            {
                var first = mission.Find(steps[0]);
                if (first.Type == "base") return first;
                var last = mission.Find(steps[steps.Count - 1]);
                if (last.Type == "base") return last;
            }
            return mission.Waypoints.FirstOrDefault(w => w.Type == "base");
        }

        // ---- background route planning ----

        private sealed class PlanJob
        {
            public string SessionId;
            public SessionState Session;
            public string TargetId;
            public Point2 From;
            public Point2 To;
            public long Generation;
            public RouteResult Result;
        }

        private BlockingCollection<PlanJob> _planJobs;
        private ConcurrentQueue<PlanJob> _planResults;
        private Thread _planWorker;

        private void StartPlanWorker(WalkingModel model, RouteOptions routeOptions)
        {
            // The worker owns its own router: AStarRouter reuses arrays and is not thread-safe.
            var workerRouter = new AStarRouter(_map, model, routeOptions);
            _planJobs = new BlockingCollection<PlanJob>(MaxPendingPlans);
            _planResults = new ConcurrentQueue<PlanJob>();
            _planWorker = new Thread(() =>
            {
                foreach (var job in _planJobs.GetConsumingEnumerable())
                {
                    // A session evicted while its job waited in the queue no longer needs the route.
                    if (!job.Session.Evicted)
                    {
                        try
                        {
                            job.Result = workerRouter.Find(job.From, job.To);
                        }
                        catch (Exception)
                        {
                            // A failed search must not kill the worker; the session simply reports no route.
                            job.Result = RouteResult.NotFound(0, 0);
                        }
                    }
                    _planResults.Enqueue(job);
                }
            })
            {
                IsBackground = true,
                Name = "Marps route planner",
            };
            _planWorker.Start();
        }

        /// <summary>Upper bound on queued route searches, so churn in sessions cannot grow the queue without limit.</summary>
        private const int MaxPendingPlans = 256;

        /// <summary>
        /// Queues a route search without blocking. When the queue is full nothing is queued and the session
        /// asks again on a later call.
        /// </summary>
        private void EnqueuePlan(string sessionId, SessionState session, Point2 from, Waypoint target)
        {
            bool queued = _planJobs.TryAdd(new PlanJob
            {
                SessionId = sessionId,
                Session = session,
                TargetId = target.Id,
                From = from,
                To = target.Position,
                Generation = session.PlanGeneration,
            });
            session.PlanPending = queued;
        }

        /// <summary>
        /// Applies the routes the worker has finished (called with the lock held).
        /// Returns true if one of them belongs to <paramref name="sessionId"/>.
        /// Results for sessions that were evicted in the meantime are dropped.
        /// </summary>
        private bool ApplyCompletedPlans(string sessionId)
        {
            if (_planResults == null) return false;
            bool forThisSession = false;
            while (_planResults.TryDequeue(out var job))
            {
                if (!_sessions.TryGetValue(job.SessionId, out var s) || !ReferenceEquals(s, job.Session)) continue;
                if (job.Result == null || job.TargetId != s.WantedTargetId || job.Generation != s.PlanGeneration)
                {
                    // Skipped search, or a route that is no longer wanted: the target changed while it ran
                    // (stops reordered, emergency) or a newer route was planned synchronously meanwhile.
                    // The session asks again on its next call if it still needs one.
                    s.PlanPending = false;
                    continue;
                }
                s.Route = job.Result;
                s.TargetId = job.TargetId;
                s.PrepareCumulative(_options.PolylineToleranceM);
                s.PlanPending = false;
                if (job.SessionId == sessionId) forThisSession = true;
            }
            return forThisSession;
        }

        /// <summary>Stops the background planner (no effect when background replanning is off).</summary>
        public void Dispose()
        {
            if (_planJobs == null) return;
            _planJobs.CompleteAdding();
            _planWorker.Join(TimeSpan.FromSeconds(5));
            _planJobs.Dispose();
            _planJobs = null;
        }

        /// <summary>Direct route between two points, without a session (for planning from tools).</summary>
        public RouteResult PlanRoute(Point2 from, Point2 to)
        {
            lock (_lock) return _router.Find(from, to);
        }

        /// <summary>
        /// Plans every leg of a visiting order, starting at <paramref name="from"/> and going through the given
        /// places in sequence (for previewing a reordered mission). Throws if an id is not a known place.
        /// </summary>
        public List<ItineraryLeg> PlanItinerary(Point2 from, IEnumerable<string> placeIds)
        {
            if (placeIds == null) throw new ArgumentNullException(nameof(placeIds));
            var targets = new List<Waypoint>();
            foreach (var id in placeIds)
                targets.Add(_mission.Find(id) ?? throw new ArgumentException($"Unknown place '{id}'.", nameof(placeIds)));

            var legs = new List<ItineraryLeg>(targets.Count);
            lock (_lock)
            {
                Point2 current = from;
                foreach (var target in targets)
                {
                    legs.Add(new ItineraryLeg { TargetId = target.Id, From = current, Route = _router.Find(current, target.Position) });
                    current = target.Position;
                }
            }
            return legs;
        }

        /// <summary>
        /// Reorders the stops still ahead in a session (see <see cref="NavigationRequest.StopOrder"/>).
        /// Returns null on success or the reason the order was rejected (the order is then left unchanged).
        /// Called with the lock held.
        /// </summary>
        private string ApplyStopOrder(SessionState s, List<string> requested)
        {
            var order = s.Order;
            int lo = Math.Max(s.Step, IsBaseStep(order, 0) ? 1 : 0);   // the departure base stays first
            int hi = order.Count - 1;
            if (order.Count > 1 && IsBaseStep(order, hi)) hi--;          // the return to base stays last
            int count = Math.Max(0, hi - lo + 1);

            var pool = count > 0 ? order.GetRange(lo, count) : new List<string>();
            var head = new List<string>(count);
            var seen = new HashSet<string>();
            foreach (var id in requested)
            {
                if (string.IsNullOrEmpty(id)) return "Empty stop id in the new order.";
                if (!seen.Add(id)) return $"Stop '{id}' is listed twice.";
                if (!_mission.ObjectiveIds.Contains(id)) return $"'{id}' is not a stop of this mission.";
                int at = pool.IndexOf(id);
                if (at < 0) continue;                                    // already done or a fixed base: ignored
                head.Add(id);
                pool.RemoveAt(at);
            }
            head.AddRange(pool);
            if (count > 0)
            {
                order.RemoveRange(lo, count);
                order.InsertRange(lo, head);
            }
            return null;
        }

        private bool IsBaseStep(List<string> order, int index) =>
            index >= 0 && index < order.Count && _mission.Find(order[index]).Type == "base";

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
                    session = new SessionState { Order = new List<string>(_mission.ObjectiveIds) };
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

                var steps = session.Order;
                var commandAlerts = new List<AlertInfo>();

                // 0. Commands: emergency on / off and a new stop order.
                bool emergencyStarted = false;
                if (req.CancelEmergency) session.Emergency = false;
                if (req.EmergencyReturn && !session.Emergency)
                {
                    if (_base == null)
                    {
                        commandAlerts.Add(new AlertInfo
                        {
                            Code = "NO_BASE",
                            Severity = "danger",
                            Message = "The mission has no base: emergency return is not available.",
                        });
                    }
                    else
                    {
                        session.Emergency = true;
                        session.EmergencySinceS = req.MissionTimeS;
                        emergencyStarted = true;
                    }
                }
                if (req.StopOrder != null)
                {
                    string error = ApplyStopOrder(session, req.StopOrder);
                    if (error != null)
                        commandAlerts.Add(new AlertInfo { Code = "INVALID_STOP_ORDER", Severity = "warning", Message = error });
                }

                // 1. Progress by proximity: the current step is completed if the path travelled since the last
                //    sample passes within ReachRadiusM of its point (so a slow sampling rate cannot skip it).
                // Only the first pending step is checked against the travelled path; later steps are checked by
                // proximity to the current position (otherwise the departure base would be credited again on return).
                Point2 previous = session.HasLast ? session.LastPos : pos;
                bool firstStep = true;
                // In an emergency nothing is credited: passing by a stop on the way home does not complete it.
                while (!session.Emergency && session.Step < steps.Count)
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
                if (!session.Emergency && !string.IsNullOrEmpty(req.ActiveWaypointId))
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

                Waypoint target = session.Emergency
                    ? _base
                    : (session.Step < steps.Count ? _mission.Find(steps[session.Step]) : null);
                session.WantedTargetId = target?.Id;

                // 3. Route: recalculated if the target changed, if requested, or if the astronaut moved away from the route.
                bool replanned = ApplyCompletedPlans(sid);
                if (target != null)
                {
                    bool needPlan = session.Route == null || session.TargetId != target.Id || req.ForceReplan || emergencyStarted;
                    if (!needPlan)
                    {
                        session.Nearest(pos, out _, out double offDist);
                        needPlan = offDist > _options.OffRouteM;
                    }
                    // The emergency route is computed at once, even in background mode: the HUD must not wait for it.
                    if (needPlan && _options.BackgroundReplanning && !emergencyStarted)
                    {
                        // Do not block: the search runs on the worker thread and is applied on a later call.
                        // Meanwhile the previous route (if any) is returned with Pending = true.
                        if (!session.PlanPending) EnqueuePlan(sid, session, pos, target);
                    }
                    else if (needPlan)
                    {
                        session.Route = _router.Find(pos, target.Position);
                        session.TargetId = target.Id;
                        session.PlanGeneration++;           // results of searches queued before this one are stale
                        session.PrepareCumulative(_options.PolylineToleranceM);
                        replanned = true;
                    }

                    response.Route = BuildRouteInfo(session, target, pos, replanned);
                    response.Route.Pending = session.PlanPending;
                    response.NextWaypoint = BuildNextWaypoint(target, pos, req.HeadingDeg);
                }
                else
                {
                    response.MissionComplete = true;
                    session.Route = null;
                    session.TargetId = null;
                    session.PrepareCumulative(_options.PolylineToleranceM);
                }

                // In an emergency the remaining stops stay pending (they resume on cancel); the return step is active.
                int last = steps.Count - 1;
                bool returnStepActive = session.Emergency && last >= session.Step && steps[last] == _base.Id;
                for (int k = 0; k < steps.Count; k++)
                {
                    var w = _mission.Find(steps[k]);
                    string status;
                    if (k < session.Step) status = "done";
                    else if (session.Emergency) status = returnStepActive && k == last ? "active" : "pending";
                    else status = k == session.Step ? "active" : "pending";
                    response.Objectives.Add(new ObjectiveInfo
                    {
                        Step = k,
                        Id = w.Id,
                        Name = w.Name,
                        X = w.Position.X,
                        Z = w.Position.Z,
                        Status = status,
                    });
                }

                response.Hazards = NearbyHazards(pos);
                response.Layers = BuildLayers(pos, out int visible);
                response.Layers.VisibleAntennas = visible;
                if (session.Emergency)
                {
                    response.Emergency = new EmergencyInfo
                    {
                        Active = true,
                        BaseId = _base.Id,
                        Arrived = pos.DistanceTo(_base.Position) <= _options.ReachRadiusM,
                        ActivatedAtS = session.EmergencySinceS,
                    };
                }

                response.Alerts = BuildAlerts(req, pos, response, target != null);
                response.Alerts.AddRange(commandAlerts);

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

            // Send the astronaut's position followed by the remaining part of the precomputed simplified route.
            var simplified = s.Simplified;
            int from = simplified.Count == 0 ? 0 : NearestIndex(simplified, pos);
            var points = new List<Point2>(simplified.Count - from + 1) { pos };
            if (simplified.Count > 0) points.AddRange(simplified.GetRange(from, simplified.Count - from));
            info.Points = points;
            return info;
        }

        /// <summary>Index of the point in the list closest to p (the list must not be empty).</summary>
        private static int NearestIndex(List<Point2> points, Point2 p)
        {
            int best = 0;
            double bestSq = double.MaxValue;
            for (int i = 0; i < points.Count; i++)
            {
                double dx = points[i].X - p.X, dz = points[i].Z - p.Z;
                double sq = dx * dx + dz * dz;
                if (sq < bestSq)
                {
                    bestSq = sq;
                    best = i;
                }
            }
            return best;
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

        private const int MaxNearbyHazards = 10;

        /// <summary>The closest hazard zones within HazardRadiusM, nearest first (at most 10).</summary>
        private List<HazardInfo> NearbyHazards(Point2 pos)
        {
            // Insertion into a short sorted list: no LINQ, no temporary objects for zones that are out of range.
            var result = new List<HazardInfo>(MaxNearbyHazards);
            var zones = _mission.HazardZones;
            for (int i = 0; i < zones.Count; i++)
            {
                var zone = zones[i];
                double d = zone.DistanceTo(pos);
                if (d > _options.HazardRadiusM) continue;
                if (result.Count == MaxNearbyHazards && d >= result[result.Count - 1].DistanceM) continue;

                int at = result.Count;
                while (at > 0 && result[at - 1].DistanceM > d) at--;
                result.Insert(at, ToHazardInfo(zone, d));
                if (result.Count > MaxNearbyHazards) result.RemoveAt(result.Count - 1);
            }
            return result;
        }

        private static HazardInfo ToHazardInfo(HazardZone zone, double distanceM) => new HazardInfo
        {
            Type = zone.Type,
            Severity = zone.Severity,
            DistanceM = distanceM,
            CenterX = zone.Center.X,
            CenterZ = zone.Center.Z,
            BoxMinX = zone.BoxMin.X,
            BoxMinZ = zone.BoxMin.Z,
            BoxMaxX = zone.BoxMax.X,
            BoxMaxZ = zone.BoxMax.Z,
            MaxSlopeDeg = zone.MaxSlopeDeg,
            AreaM2 = zone.AreaM2,
        };

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

            if (resp.Emergency != null)
            {
                var r = resp.Route;
                alerts.Add(resp.Emergency.Arrived
                    ? new AlertInfo { Code = "EMERGENCY_RETURN", Severity = "info", Message = "Emergency: arrived at base." }
                    : new AlertInfo
                    {
                        Code = "EMERGENCY_RETURN",
                        Severity = "danger",
                        Message = r != null && r.Found
                            ? $"Emergency: return to base {resp.Emergency.BaseId}, {r.RemainingM:0} m, about {r.EstimatedMinutes:0} min."
                            : $"Emergency: return to base {resp.Emergency.BaseId}.",
                    });
            }

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

            if (hasTarget && resp.Route != null && !resp.Route.Found && !resp.Route.Pending)
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
            if (oldestId != null)
            {
                _sessions[oldestId].Evicted = true;
                _sessions.Remove(oldestId);
            }
        }

        private sealed class SessionState
        {
            /// <summary>Value of the engine's use counter when this session was last used.</summary>
            public long LastUsed { get; set; }

            /// <summary>True while a background search for this session is running.</summary>
            public bool PlanPending { get; set; }

            /// <summary>Set when the session is discarded from the cache. Read by the worker thread.</summary>
            public volatile bool Evicted;

            /// <summary>Index of the current step in <see cref="Order"/>. Previous steps are completed.</summary>
            public int Step { get; set; }

            /// <summary>This session's step sequence (starts as the mission order; the astronaut may reorder it).</summary>
            public List<string> Order { get; set; }

            /// <summary>Emergency return to base is active.</summary>
            public bool Emergency { get; set; }
            public double EmergencySinceS { get; set; }

            /// <summary>Target the session needs a route to right now (background results for others are dropped).</summary>
            public string WantedTargetId { get; set; }

            /// <summary>Bumped on each synchronous plan, so older background results are not applied over it.</summary>
            public long PlanGeneration { get; set; }
            public Point2 LastPos { get; set; }
            public bool HasLast { get; set; }
            public string TargetId { get; set; }
            public RouteResult Route { get; set; }
            public List<double> Cumulative { get; } = new List<double>();

            /// <summary>The route simplified once for the HUD (computed when the route is planned).</summary>
            public List<Point2> Simplified { get; private set; } = new List<Point2>();

            public void PrepareCumulative(double simplifyToleranceM)
            {
                Cumulative.Clear();
                Simplified = new List<Point2>();
                if (Route == null || !Route.Found) return;
                double acc = 0;
                for (int i = 0; i < Route.Points.Count; i++)
                {
                    if (i > 0) acc += Route.Points[i - 1].DistanceTo(Route.Points[i]);
                    Cumulative.Add(acc);
                }
                Simplified = PathSimplifier.Simplify(Route.Points, simplifyToleranceM);
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
