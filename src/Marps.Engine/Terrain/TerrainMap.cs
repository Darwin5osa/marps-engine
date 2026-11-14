using System;
using Marps.Engine.Geometry;

namespace Marps.Engine.Terrain
{
    /// <summary>
    /// Grid of heights (meters) and hazards (0 walkable, 1 slope 15-22°, 2 slope &gt;22°, 3 rough zone).
    /// Row 0 = north edge. Index = row * Width + column.
    /// Each node's slope is precomputed once; later queries are O(1).
    /// </summary>
    public sealed class TerrainMap
    {
        private const double RadToDeg = 180.0 / Math.PI;

        public TerrainMap(TerrainMetadata meta, float[] heights, byte[] hazard)
        {
            if (meta == null) throw new ArgumentNullException(nameof(meta));
            if (meta.Width < 2 || meta.Height < 2)
                throw new ArgumentException("The grid needs at least 2x2 points.", nameof(meta));

            int n = meta.Width * meta.Height;
            if (heights == null || heights.Length != n)
                throw new ArgumentException($"Expected {n} heights.", nameof(heights));
            if (hazard == null || hazard.Length != n)
                throw new ArgumentException($"Expected {n} hazard values.", nameof(hazard));

            // Reject corrupt data here instead of letting it slip into slope checks: NaN makes every
            // comparison false, and an unknown hazard code would be treated as walkable.
            for (int i = 0; i < n; i++)
            {
                if (!float.IsFinite(heights[i]))
                    throw new ArgumentException($"Height at index {i} is not a finite number.", nameof(heights));
                if (hazard[i] > 3)
                    throw new ArgumentException($"Hazard code {hazard[i]} at index {i} is not in 0..3.", nameof(hazard));
            }

            Meta = meta;
            Width = meta.Width;
            Height = meta.Height;
            Cell = meta.CellM;
            Heights = heights;
            Hazard = hazard;
            SlopeDeg = ComputeSlopes();
        }

        public TerrainMetadata Meta { get; }
        public int Width { get; }
        public int Height { get; }
        public double Cell { get; }
        public float[] Heights { get; }
        public byte[] Hazard { get; }

        /// <summary>Slope in degrees of each node (central differences).</summary>
        public float[] SlopeDeg { get; }

        public int Index(int c, int r) => r * Width + c;

        public bool InBounds(int c, int r) => c >= 0 && r >= 0 && c < Width && r < Height;

        /// <summary>Local position of the node (column c, row r).</summary>
        public Point2 NodePosition(int c, int r) => new Point2(c * Cell, (Height - 1 - r) * Cell);

        /// <summary>True if the point falls inside the map square.</summary>
        public bool IsInside(Point2 p) =>
            p.X >= 0 && p.Z >= 0 && p.X <= (Width - 1) * Cell && p.Z <= (Height - 1) * Cell;

        /// <summary>Nearest node to a point (clamped to the edge if the point is outside).</summary>
        public void NearestNode(Point2 p, out int c, out int r)
        {
            c = Clamp((int)Math.Round(p.X / Cell), 0, Width - 1);
            r = Clamp((int)Math.Round((Height - 1) - p.Z / Cell), 0, Height - 1);
        }

        /// <summary>Altura interpolada bilinealmente en un punto local (recortada al mapa).</summary>
        public double ElevationAt(Point2 p)
        {
            double fx = Clamp(p.X / Cell, 0, Width - 1);
            double fr = (Height - 1) - Clamp(p.Z / Cell, 0, Height - 1);

            int c0 = Math.Min((int)fx, Width - 2);
            int r0 = Math.Min((int)fr, Height - 2);
            double tc = fx - c0;
            double tr = fr - r0;

            double h00 = Heights[Index(c0, r0)];
            double h10 = Heights[Index(c0 + 1, r0)];
            double h01 = Heights[Index(c0, r0 + 1)];
            double h11 = Heights[Index(c0 + 1, r0 + 1)];

            return (1 - tr) * ((1 - tc) * h00 + tc * h10) + tr * ((1 - tc) * h01 + tc * h11);
        }

        /// <summary>Slope in degrees at a point, taken from the nearest node.</summary>
        public double SlopeAt(Point2 p)
        {
            NearestNode(p, out int c, out int r);
            return SlopeDeg[Index(c, r)];
        }

        private float[] ComputeSlopes()
        {
            var slopes = new float[Width * Height];
            for (int r = 0; r < Height; r++)
            {
                int r1 = Math.Max(r - 1, 0), r2 = Math.Min(r + 1, Height - 1);
                for (int c = 0; c < Width; c++)
                {
                    int c1 = Math.Max(c - 1, 0), c2 = Math.Min(c + 1, Width - 1);
                    double dx = (Heights[Index(c2, r)] - Heights[Index(c1, r)]) / ((c2 - c1) * Cell);
                    double dz = (Heights[Index(c, r2)] - Heights[Index(c, r1)]) / ((r2 - r1) * Cell);
                    slopes[Index(c, r)] = (float)(Math.Atan(Math.Sqrt(dx * dx + dz * dz)) * RadToDeg);
                }
            }
            return slopes;
        }

        private static int Clamp(int v, int lo, int hi) => v < lo ? lo : (v > hi ? hi : v);

        private static double Clamp(double v, double lo, double hi) => v < lo ? lo : (v > hi ? hi : v);
    }
}
