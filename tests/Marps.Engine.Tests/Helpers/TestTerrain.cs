using System;
using Marps.Engine.Terrain;

namespace Marps.Engine.Tests.Helpers
{
    /// <summary>Generates synthetic terrains for tests: heights from a function and hazard derived from slope.</summary>
    public static class TestTerrain
    {
        /// <summary>
        /// Map of n x n nodes with cell <paramref name="cell"/>, origin (0,0).
        /// height(x, z) gives the height in meters. Hazard is computed with the same rules as the real data
        /// (0 &lt;=15°, 1 15-22°, 2 &gt;22°) unless stated otherwise.
        /// </summary>
        public static TerrainMap Build(int n, double cell, Func<double, double, double> height)
        {
            var meta = new TerrainMetadata
            {
                Width = n,
                Height = n,
                OriginX = 0,
                OriginY = 0,
                SideM = cell * (n - 1),
                Projection = "test",
                HeightsFile = "terrain.f32",
                HazardFile = "hazard.u8",
            };

            var heights = new float[n * n];
            var zero = new byte[n * n];
            for (int r = 0; r < n; r++)
            {
                for (int c = 0; c < n; c++)
                {
                    double x = c * cell;
                    double z = (n - 1 - r) * cell; // fila 0 = norte
                    heights[r * n + c] = (float)height(x, z);
                }
            }

            // First pass to measure slopes and derive the hazard.
            var probe = new TerrainMap(meta, heights, zero);
            var hazard = new byte[n * n];
            for (int i = 0; i < hazard.Length; i++)
            {
                float s = probe.SlopeDeg[i];
                hazard[i] = (byte)(s > 22f ? 2 : (s >= 15f ? 1 : 0));
            }
            return new TerrainMap(meta, heights, hazard);
        }

        /// <summary>Flat terrain with hazard 0 everywhere.</summary>
        public static TerrainMap Flat(int n, double cell = 2.0)
        {
            var map = Build(n, cell, (x, z) => 0.0);
            return WithHazard(map, (c, r) => 0);
        }

        /// <summary>Copies the map, changing the hazard node by node.</summary>
        public static TerrainMap WithHazard(TerrainMap map, Func<int, int, byte> hazardAt)
        {
            var hazard = new byte[map.Width * map.Height];
            for (int r = 0; r < map.Height; r++)
                for (int c = 0; c < map.Width; c++)
                    hazard[map.Index(c, r)] = hazardAt(c, r);
            return new TerrainMap(map.Meta, map.Heights, hazard);
        }
    }
}
