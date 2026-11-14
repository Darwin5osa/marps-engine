using System;
using System.Collections.Generic;
using Marps.Engine.Geometry;

namespace Marps.Engine.Routing
{
    /// <summary>Douglas-Peucker simplification (iterative) to reduce the points sent to the HUD.</summary>
    public static class PathSimplifier
    {
        public static List<Point2> Simplify(IReadOnlyList<Point2> pts, double toleranceM)
        {
            int n = pts.Count;
            if (n < 3) return new List<Point2>(pts);

            var keep = new bool[n];
            keep[0] = true;
            keep[n - 1] = true;

            var stack = new Stack<(int A, int B)>();
            stack.Push((0, n - 1));
            while (stack.Count > 0)
            {
                var (a, b) = stack.Pop();
                double maxD = -1;
                int idx = -1;
                for (int i = a + 1; i < b; i++)
                {
                    double d = DistanceToSegment(pts[i], pts[a], pts[b]);
                    if (d > maxD)
                    {
                        maxD = d;
                        idx = i;
                    }
                }
                if (idx >= 0 && maxD > toleranceM)
                {
                    keep[idx] = true;
                    stack.Push((a, idx));
                    stack.Push((idx, b));
                }
            }

            var result = new List<Point2>();
            for (int i = 0; i < n; i++)
                if (keep[i]) result.Add(pts[i]);
            return result;
        }

        public static double DistanceToSegment(Point2 p, Point2 a, Point2 b)
        {
            double vx = b.X - a.X, vz = b.Z - a.Z;
            double len2 = vx * vx + vz * vz;
            if (len2 == 0) return p.DistanceTo(a);
            double t = ((p.X - a.X) * vx + (p.Z - a.Z) * vz) / len2;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            var proj = new Point2(a.X + t * vx, a.Z + t * vz);
            return p.DistanceTo(proj);
        }
    }
}
