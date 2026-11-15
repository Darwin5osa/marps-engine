using Marps.Engine.Abstractions;
using Marps.Engine.Terrain;

namespace Marps.Engine.Data
{
    /// <summary>Terrain provider that reads the processed data folder.</summary>
    public sealed class FileTerrainProvider : ITerrainProvider
    {
        private readonly string _directory;

        public FileTerrainProvider(string directory)
        {
            _directory = directory;
        }

        public TerrainMap Load() => TerrainFiles.Load(_directory);
    }
}
