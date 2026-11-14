using System;
using System.IO;
using Marps.Engine.Mission;
using Marps.Engine.Terrain;

namespace Marps.Engine.Tests.Helpers
{
    /// <summary>
    /// Real Deuteronilus Mensae data (context/processed + context/*.json).
    /// If they are missing (e.g. in a clean clone), the tests that use them are marked as skipped.
    /// To generate context/processed: dotnet run --project tools/Marps.DataImport -- context
    /// </summary>
    public static class RealData
    {
        private static readonly Lazy<string> _root = new Lazy<string>(FindContextDir);
        private static readonly Lazy<TerrainMap> _terrain = new Lazy<TerrainMap>(() => TerrainFiles.Load(ProcessedDir));
        private static readonly Lazy<MissionDefinition> _mission = new Lazy<MissionDefinition>(() =>
            MissionLoader.Load(Path.Combine(ContextDir, "mission.json"), Path.Combine(ContextDir, "hazards.json"), _terrain.Value.Meta));

        public static bool Available => Directory.Exists(ProcessedDir) && File.Exists(Path.Combine(ProcessedDir, "terrain.json"));

        public static string ContextDir => _root.Value;

        public static string ProcessedDir => Path.Combine(ContextDir, "processed");

        public static TerrainMap Terrain => _terrain.Value;

        public static MissionDefinition Mission => _mission.Value;

        private static string FindContextDir()
        {
            string env = Environment.GetEnvironmentVariable("MARPS_DATA");
            if (!string.IsNullOrEmpty(env)) return env;

            var dir = new DirectoryInfo(AppContext.BaseDirectory);
            while (dir != null)
            {
                string candidate = Path.Combine(dir.FullName, "context");
                if (Directory.Exists(Path.Combine(candidate, "processed"))) return candidate;
                dir = dir.Parent;
            }
            return Path.Combine(AppContext.BaseDirectory, "context");
        }
    }
}
