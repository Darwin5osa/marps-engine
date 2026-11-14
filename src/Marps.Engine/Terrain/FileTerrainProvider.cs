using Marps.Engine.Abstractions;

namespace Marps.Engine.Terrain
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
