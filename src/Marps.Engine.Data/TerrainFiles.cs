using System;
using System.IO;
using System.Text.Json;
using Marps.Engine.Terrain;

namespace Marps.Engine.Data
{
    /// <summary>
    /// Loads a map from a folder containing:
    ///   terrain.json   (metadata, <see cref="TerrainMetadata"/>)
    ///   terrain.f32    (float32 little-endian heights, row 0 = north)
    ///   hazard.u8      (hazard per node, 1 byte)
    /// These binaries are generated from the TIFFs by Marps.DataImport.
    /// </summary>
    public static class TerrainFiles
    {
        public static TerrainMetadata ReadMetadata(string jsonPath)
        {
            string json = File.ReadAllText(jsonPath);
            return JsonSerializer.Deserialize<TerrainMetadata>(json, MarpsJson.Options)
                   ?? throw new InvalidDataException($"Could not read {jsonPath}.");
        }

        public static TerrainMap Load(string directory)
        {
            var meta = ReadMetadata(Path.Combine(directory, "terrain.json"));
            return Load(meta, directory);
        }

        public static TerrainMap Load(TerrainMetadata meta, string directory)
        {
            if (meta == null) throw new ArgumentNullException(nameof(meta));
            // Checked before multiplying: a corrupt terrain.json must not overflow the expected file sizes.
            if (meta.Width < 2 || meta.Height < 2 || (long)meta.Width * meta.Height * 4 > int.MaxValue)
                throw new InvalidDataException($"terrain.json: grid {meta.Width}x{meta.Height} is not valid.");
            int n = meta.Width * meta.Height;
            float[] heights = ReadFloat32(Path.Combine(directory, meta.HeightsFile ?? "terrain.f32"), n);
            byte[] hazard = File.ReadAllBytes(RequireFile(Path.Combine(directory, meta.HazardFile ?? "hazard.u8"), n));
            if (hazard.Length != n)
                throw new InvalidDataException($"hazard.u8 has {hazard.Length} bytes, expected {n}.");
            return new TerrainMap(meta, heights, hazard);
        }

        /// <summary>Reads n little-endian floats. Fails if the file is not exactly the expected size.</summary>
        public static float[] ReadFloat32(string path, int count)
        {
            RequireFile(path, count * 4);
            byte[] bytes = File.ReadAllBytes(path);
            if (!BitConverter.IsLittleEndian)
                throw new PlatformNotSupportedException("The reader assumes a little-endian host.");
            var values = new float[count];
            Buffer.BlockCopy(bytes, 0, values, 0, bytes.Length);
            return values;
        }

        private static string RequireFile(string path, int expectedBytes)
        {
            if (!File.Exists(path)) throw new FileNotFoundException("Terrain file is missing.", path);
            long size = new FileInfo(path).Length;
            if (size != expectedBytes)
                throw new InvalidDataException($"{path}: {size} bytes, expected {expectedBytes}.");
            return path;
        }
    }
}
