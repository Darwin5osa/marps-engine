using System;
using Marps.Engine.Geometry;

namespace Marps.Engine.Terrain
{
    /// <summary>
    /// Grid of heights (meters) and hazards (0 walkable, 1 slope 15-22°, 2 slope &gt;22°, 3 rough zone).
    /// Row 0 = north edge. Index = row * Width + column.
    /// Each node's slope is precomputed once; later queries are O(1).
    /// Immutable: the constructor copies the arrays it is given, and the grid is only readable from outside
    /// (per node, or as copies), so nothing outside the engine can change the terrain it routes over.
    /// </summary>
    public sealed class TerrainMap
    {
        private const double RadToDeg = 180.0 / Math.PI;
        private readonly TerrainMetadata _meta;

        public TerrainMap(TerrainMetadata meta, float[] heights, byte[] hazard)
        {
            if (meta == null) throw new ArgumentNullException(nameof(meta));
            if (meta.Width < 2 || meta.Height < 2)
                throw new ArgumentException("The grid needs at least 2x2 points.", nameof(meta));
            if (!double.IsFinite(meta.CellM) || meta.CellM <= 0)
                throw new ArgumentException("SideM must be a positive finite number (it sets the cell size).", nameof(meta));

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

            _meta = meta.FrozenCopy();
            Width = meta.Width;
            Height = meta.Height;
            Cell = meta.CellM;
            // Own copies: a caller that reuses or edits its arrays afterwards cannot desync heights and slopes.
            Heights = (float[])heights.Clone();
            Hazard = (byte[])hazard.Clone();
            SlopeDeg = ComputeSlopes();
        }

        /// <summary>The map metadata (frozen: its setters throw; use <see cref="TerrainMetadata.Clone"/> to edit a copy).</summary>
        public TerrainMetadata Meta => _meta;
        public int Width { get; }
        public int Height { get; }
        public double Cell { get; }

        // The raw grids stay inside the engine (router, alerts, tests). Frontends read them per node or as copies.
        internal float[] Heights { get; }
        internal byte[] Hazard { get; }
        /// <summary>Slope in degrees of each node (central differences).</summary>
        internal float[] SlopeDeg { get; }

        public int Index(int c, int r) => r * Width + c;

        public bool InBounds(int c, int r) => c >= 0 && r >= 0 && c < Width && r < Height;

        /// <summary>Height of node (c, r) in meters.</summary>
        public float HeightAt(int c, int r) => Heights[CheckedIndex(c, r)];

        /// <summary>Hazard code of node (c, r): 0 walkable, 1 slope 15-22°, 2 slope &gt;22°, 3 rough.</summary>
        public byte HazardAt(int c, int r) => Hazard[CheckedIndex(c, r)];

        /// <summary>Slope of node (c, r) in degrees.</summary>
        public float SlopeAtNode(int c, int r) => SlopeDeg[CheckedIndex(c, r)];

        /// <summary>Copy of every height (row 0 = north), e.g. to build a texture.</summary>
        public float[] CopyHeights() => (float[])Heights.Clone();

        /// <summary>Copy of every hazard code (row 0 = north).</summary>
        public byte[] CopyHazard() => (byte[])Hazard.Clone();

        /// <summary>Copy of every node slope in degrees (row 0 = north).</summary>
        public float[] CopySlopes() => (float[])SlopeDeg.Clone();

        /// <summary>Local position of the node (column c, row r).</summary>
        public Point2 NodePosition(int c, int r) => new Point2(c * Cell, (Height - 1 - r) * Cell);

        /// <summary>True if the point falls inside the map square (false for NaN).</summary>
        public bool IsInside(Point2 p) =>
            p.X >= 0 && p.Z >= 0 && p.X <= (Width - 1) * Cell && p.Z <= (Height - 1) * Cell;

        /// <summary>
        /// Nearest node to a point (clamped to the edge if the point is outside). The clamp happens in floating
        /// point before the cast, and NaN maps to the first node, so the result never depends on how a runtime
        /// converts out-of-range doubles to int (that conversion is undefined in IL2CPP / C++).
        /// </summary>
        public void NearestNode(Point2 p, out int c, out int r)
        {
            c = (int)Math.Round(Clamp(p.X / Cell, 0, Width - 1));
            r = (int)Math.Round(Clamp((Height - 1) - p.Z / Cell, 0, Height - 1));
        }

        /// <summary>Bilinearly interpolated height at a local point (clamped to the map; NaN gives a corner).</summary>
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

        private int CheckedIndex(int c, int r)
        {
            if (!InBounds(c, r)) throw new ArgumentOutOfRangeException(nameof(c), $"Node ({c}, {r}) is outside the {Width}x{Height} grid.");
            return Index(c, r);
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

        /// <summary>Clamp where NaN gives <paramref name="lo"/> (plain comparisons would let NaN through).</summary>
        private static double Clamp(double v, double lo, double hi) => !(v >= lo) ? lo : (v > hi ? hi : v);
    }
}
