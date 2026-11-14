using System;
using System.Collections.Generic;
using System.Diagnostics;
using Marps.Engine.Geometry;
using Marps.Engine.Terrain;

namespace Marps.Engine.Routing
{
    /// <summary>
    /// A* over the grid (8 neighbors). Cost = minutes in suit (<see cref="WalkingModel"/>).
    /// Hard rules: nodes with hazard 2 are never entered, and no step may exceed the slope limit
    /// (measured between the two nodes of the step, not by sampling).
    /// Reuses its arrays between searches: one instance per thread.
    /// </summary>
    public sealed class AStarRouter
    {
        private static readonly int[] Dc = { 1, -1, 0, 0, 1, 1, -1, -1 };
        private static readonly int[] Dr = { 0, 0, 1, -1, 1, -1, 1, -1 };
        private static readonly double Sqrt2 = Math.Sqrt(2.0);

        private readonly TerrainMap _map;
        private readonly WalkingModel _model;
        private readonly RouteOptions _options;
        private readonly double _maxTan;
        private readonly double _heuristicWeight;
        private readonly float[] _g;
        private readonly int[] _parent;
        private readonly int[] _openStamp;
        private readonly int[] _closedStamp;
        private readonly MinHeap _heap;
        private int _search;

        public AStarRouter(TerrainMap map, WalkingModel model = null, RouteOptions options = null)
        {
            _map = map ?? throw new ArgumentNullException(nameof(map));
            _model = model ?? WalkingModel.Fitted;
            _options = options ?? new RouteOptions();
            _maxTan = Math.Tan(_options.MaxSlopeDeg * Math.PI / 180.0);
            _heuristicWeight = _options.HeuristicWeight;

            int n = map.Width * map.Height;
            _g = new float[n];
            _parent = new int[n];
            _openStamp = new int[n];
            _closedStamp = new int[n];
            _heap = new MinHeap(4096);
        }

        /// <summary>Route between two local points (the nearest node of each is used).</summary>
        public RouteResult Find(Point2 from, Point2 to)
        {
            _map.NearestNode(from, out int sc, out int sr);
            _map.NearestNode(to, out int gc, out int gr);
            return FindNodes(sc, sr, gc, gr);
        }

        public RouteResult FindNodes(int sc, int sr, int gc, int gr)
        {
            if (!_map.InBounds(sc, sr) || !_map.InBounds(gc, gr))
                throw new ArgumentOutOfRangeException(nameof(sc), "Node is outside the map.");

            var clock = Stopwatch.StartNew();
            int w = _map.Width;
            int h = _map.Height;
            double cell = _map.Cell;
            float[] heights = _map.Heights;
            byte[] hazard = _map.Hazard;

            int start = _map.Index(sc, sr);
            int goal = _map.Index(gc, gr);

            _search++;
            if (_search == int.MaxValue)
            {
                Array.Clear(_openStamp, 0, _openStamp.Length);
                Array.Clear(_closedStamp, 0, _closedStamp.Length);
                _search = 1;
            }
            int s = _search;

            _heap.Clear();
            _g[start] = 0f;
            _parent[start] = -1;
            _openStamp[start] = s;
            _heap.Push((float)Heuristic(start, goal, w, cell), start);

            int expansions = 0;
            bool found = false;
            while (_heap.Count > 0)
            {
                int cur = _heap.Pop();
                if (_closedStamp[cur] == s) continue;
                _closedStamp[cur] = s;
                expansions++;
                if (cur == goal)
                {
                    found = true;
                    break;
                }

                int cc = cur % w;
                int cr = cur / w;
                float gCur = _g[cur];
                for (int k = 0; k < 8; k++)
                {
                    int nc = cc + Dc[k];
                    int nr = cr + Dr[k];
                    if (nc < 0 || nr < 0 || nc >= w || nr >= h) continue;

                    int n = nr * w + nc;
                    if (_closedStamp[n] == s || hazard[n] == 2) continue;

                    bool diagonal = Dc[k] != 0 && Dr[k] != 0;
                    double horizontal = diagonal ? cell * Sqrt2 : cell;
                    double dh = heights[n] - heights[cur];
                    if (Math.Abs(dh) > _maxTan * horizontal) continue; // step slope out of limit

                    double step = _model.EdgeMinutes(horizontal, dh);
                    if (hazard[n] == 3) step *= _options.RoughFactor;

                    double tentative = gCur + step;
                    if (_openStamp[n] != s || tentative < _g[n])
                    {
                        _openStamp[n] = s;
                        _g[n] = (float)tentative;
                        _parent[n] = cur;
                        _heap.Push((float)(tentative + Heuristic(n, goal, w, cell)), n);
                    }
                }
            }

            clock.Stop();
            double ms = clock.Elapsed.TotalMilliseconds;
            if (!found) return RouteResult.NotFound(expansions, ms);

            return BuildResult(goal, w, cell, heights, expansions, ms);
        }

        private RouteResult BuildResult(int goal, int w, double cell, float[] heights, int expansions, double ms)
        {
            var chain = new List<int>();
            for (int i = goal; i != -1; i = _parent[i]) chain.Add(i);
            chain.Reverse();

            var result = new RouteResult
            {
                Found = true,
                Expansions = expansions,
                ComputeMs = ms,
                EstimatedMinutes = _g[goal],
                Points = new List<Point2>(chain.Count),
            };

            double maxSlope = 0;
            for (int i = 0; i < chain.Count; i++)
            {
                int idx = chain[i];
                result.Points.Add(_map.NodePosition(idx % w, idx / w));
                if (i == 0) continue;

                int prev = chain[i - 1];
                int dcol = Math.Abs(idx % w - prev % w);
                int drow = Math.Abs(idx / w - prev / w);
                double horizontal = (dcol == 1 && drow == 1) ? cell * Sqrt2 : cell;
                double dh = heights[idx] - heights[prev];

                result.HorizontalLengthM += horizontal;
                if (dh > 0) result.AscentM += dh; else result.DescentM -= dh;

                double slope = Math.Atan(Math.Abs(dh) / horizontal) * 180.0 / Math.PI;
                if (slope > maxSlope) maxSlope = slope;
            }
            result.MaxEdgeSlopeDeg = maxSlope;
            return result;
        }

        private double Heuristic(int node, int goal, int w, double cell)
        {
            int dc = node % w - goal % w;
            int dr = node / w - goal / w;
            return _heuristicWeight * _model.LowerBoundMinutes(Math.Sqrt((double)dc * dc + (double)dr * dr) * cell);
        }
    }
}
