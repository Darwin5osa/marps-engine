using System;
using System.IO;
using System.Text.Json;
using Marps.Engine.Json;

namespace Marps.Engine.Terrain
{
    /// <summary>
    /// Loads a map from a folder containing:
    ///   terrain.json   (metadatos, <see cref="TerrainMetadata"/>)
    ///   terrain.f32    (alturas float32 little-endian, fila 0 = norte)
    ///   hazard.u8      (hazard per node, 1 byte)
    /// These binaries are generated from the TIFFs by Marps.DataImport.
    /// </summary>
    public static class TerrainFiles
    {
        public static TerrainMetadata ReadMetadata(string jsonPath)
        {
            string json = File.ReadAllText(jsonPath);
            return JsonSerializer.Deserialize<TerrainMetadata>(json, MarpsJson.Options)
                   ?? throw new InvalidDataException($"No se pudo leer {jsonPath}");
        }

        public static TerrainMap Load(string directory)
        {
            var meta = ReadMetadata(Path.Combine(directory, "terrain.json"));
            return Load(meta, directory);
        }

        public static TerrainMap Load(TerrainMetadata meta, string directory)
        {
            int n = meta.Width * meta.Height;
            float[] heights = ReadFloat32(Path.Combine(directory, meta.HeightsFile ?? "terrain.f32"), n);
            byte[] hazard = File.ReadAllBytes(RequireFile(Path.Combine(directory, meta.HazardFile ?? "hazard.u8"), n));
            if (hazard.Length != n)
                throw new InvalidDataException($"hazard.u8 tiene {hazard.Length} bytes, se esperaban {n}.");
            return new TerrainMap(meta, heights, hazard);
        }

        /// <summary>Reads n little-endian floats. Fails if the file is not exactly the expected size.</summary>
        public static float[] ReadFloat32(string path, int count)
        {
            RequireFile(path, count * 4);
            byte[] bytes = File.ReadAllBytes(path);
            if (!BitConverter.IsLittleEndian)
                throw new PlatformNotSupportedException("El lector asume host little-endian.");
            var values = new float[count];
            Buffer.BlockCopy(bytes, 0, values, 0, bytes.Length);
            return values;
        }

        private static string RequireFile(string path, int expectedBytes)
        {
            if (!File.Exists(path)) throw new FileNotFoundException("Terrain file is missing.", path);
            long size = new FileInfo(path).Length;
            if (size != expectedBytes)
                throw new InvalidDataException($"{path}: {size} bytes, se esperaban {expectedBytes}.");
            return path;
        }
    }
}
