using System;
using System.Diagnostics;
using Marps.Engine.Geometry;
using Marps.Engine.Terrain;

namespace Marps.Engine.Routing
{
    /// <summary>
    /// Walking minutes from every node of the map to one target, over the best safe route
    /// (same walking model and hard rules as the router: no blocked node is entered, no step above the slope limit).
    /// Built once with a reverse Dijkstra; every query afterwards is a lookup, cheap enough for every frame.
    /// Uses: the O₂ point of no return (minutes to the base from here) and the walk-back contour
    /// (nodes where <see cref="MinutesAtNode"/> fits in the remaining O₂). Immutable and thread-safe.
    /// The values are the true optimum, so they can be up to ~1 % lower than the time of the route
    /// <see cref="Abstractions.INavigationEngine.PlanRoute"/> returns (that search trades a little optimality for speed).
    /// </summary>
    public sealed class CostField
    {
        private readonly float[] _minutes;   // +infinity = no safe route to the target

        internal CostField(string targetId, Point2 target, int width, int height, double cell,
            float[] minutes, int reachableNodes, double computeMs)
        {
            TargetId = targetId;
            Target = target;
            Width = width;
            Height = height;
            Cell = cell;
            _minutes = minutes;
            ReachableNodes = reachableNodes;
            ComputeMs = computeMs;
        }

        /// <summary>Id of the target place, or null when the field was built for a free point.</summary>
        public string TargetId { get; }

        /// <summary>The target point as given.</summary>
        public Point2 Target { get; }

        public int Width { get; }
        public int Height { get; }
        public double Cell { get; }

        /// <summary>Nodes that have a safe route to the target (the target included).</summary>
        public int ReachableNodes { get; }

        /// <summary>Time it took to build the field.</summary>
        public double ComputeMs { get; }

        /// <summary>
        /// Minutes from the node nearest to <paramref name="p"/> to the target, or +infinity if there is no safe route.
        /// A point outside the map uses the nearest edge node (like the router); a non-finite point gives +infinity.
        /// </summary>
        public double MinutesAt(Point2 p)
        {
            if (!double.IsFinite(p.X) || !double.IsFinite(p.Z)) return double.PositiveInfinity;
            int c = (int)Math.Round(Clamp(p.X / Cell, Width - 1));
            int r = (int)Math.Round(Clamp((Height - 1) - p.Z / Cell, Height - 1));
            return _minutes[r * Width + c];
        }

        /// <summary>True if there is a safe route from the node nearest to <paramref name="p"/> to the target.</summary>
        public bool IsReachable(Point2 p) => !double.IsPositiveInfinity(MinutesAt(p));

        /// <summary>True if the walk from <paramref name="p"/> to the target fits in <paramref name="availableMinutes"/>.</summary>
        public bool CanReachWithin(Point2 p, double availableMinutes) => MinutesAt(p) <= availableMinutes;

        /// <summary>Minutes from node (c, r) to the target (row 0 = north), or +infinity.</summary>
        public double MinutesAtNode(int c, int r)
        {
            if (c < 0 || r < 0 || c >= Width || r >= Height)
                throw new ArgumentOutOfRangeException(nameof(c), $"Node ({c}, {r}) is outside the {Width}x{Height} grid.");
            return _minutes[r * Width + c];
        }

        /// <summary>Copy of every node value (row 0 = north, +infinity = unreachable), e.g. to draw the contour.</summary>
        public float[] CopyMinutes() => (float[])_minutes.Clone();

        private static double Clamp(double v, double hi) => !(v >= 0) ? 0 : (v > hi ? hi : v);

        private static readonly int[] Dc = { 1, -1, 0, 0, 1, 1, -1, -1 };
        private static readonly int[] Dr = { 0, 0, 1, -1, 1, -1, 1, -1 };

        /// <summary>
        /// Reverse Dijkstra from the target. Popping node v (its time to the target is final), each neighbor u gets
        /// time(v) + cost of the step u → v, with the router's rules for that step: v is not blocked (hazard 2)
        /// and the slope between u and v is within the limit. Up and down cost differently, so the step is
        /// always costed in the direction the astronaut walks (u to v), not the direction of the search.
        /// </summary>
        internal static CostField Build(TerrainMap map, WalkingModel model, RouteOptions options, string targetId, Point2 target)
        {
            var clock = Stopwatch.StartNew();
            int w = map.Width, h = map.Height, n = w * h;
            double cell = map.Cell;
            double maxTan = Math.Tan(options.MaxSlopeDeg * Math.PI / 180.0);
            double diagonal = cell * Math.Sqrt(2.0);
            float[] heights = map.Heights;
            byte[] hazard = map.Hazard;

            var minutes = new float[n];
            for (int i = 0; i < n; i++) minutes[i] = float.PositiveInfinity;
            var closed = new bool[n];
            var heap = new MinHeap(4096);

            map.NearestNode(target, out int tc, out int tr);
            int goal = map.Index(tc, tr);
            minutes[goal] = 0f;
            heap.Push(0f, goal);

            int reachable = 0;
            while (heap.Count > 0)
            {
                int v = heap.Pop();
                if (closed[v]) continue;
                closed[v] = true;
                reachable++;
                if (hazard[v] == 2) continue;          // nobody may step into a blocked node (only the target can be one)

                int vc = v % w, vr = v / w;
                double timeV = minutes[v];
                double roughV = hazard[v] == 3 ? options.RoughFactor : 1.0;
                for (int k = 0; k < 8; k++)
                {
                    int uc = vc + Dc[k], ur = vr + Dr[k];
                    if (uc < 0 || ur < 0 || uc >= w || ur >= h) continue;
                    int u = ur * w + uc;
                    if (closed[u]) continue;

                    double horizontal = Dc[k] != 0 && Dr[k] != 0 ? diagonal : cell;
                    double dh = heights[v] - heights[u];                 // the astronaut walks u -> v
                    if (Math.Abs(dh) > maxTan * horizontal) continue;
                    double t = timeV + model.EdgeMinutes(horizontal, dh) * roughV;
                    if (t < minutes[u])
                    {
                        minutes[u] = (float)t;
                        heap.Push((float)t, u);
                    }
                }
            }
            clock.Stop();
            return new CostField(targetId, target, w, h, cell, minutes, reachable, clock.Elapsed.TotalMilliseconds);
        }
    }
}
