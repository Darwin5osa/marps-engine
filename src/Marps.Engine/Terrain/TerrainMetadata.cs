using System;
using Marps.Engine.Geometry;

namespace Marps.Engine.Terrain
{
    /// <summary>
    /// Map metadata (terrain.json). Projection coordinates (X, Y) are in meters
    /// of the Mars equirectangular projection; the engine works in local meters.
    /// A <see cref="TerrainMap"/> keeps a frozen copy: setting a property of <see cref="TerrainMap.Meta"/> throws.
    /// </summary>
    public sealed class TerrainMetadata
    {
        private int _width, _height;
        private double _originX, _originY, _sideM, _zMin, _zMax;
        private string _projection, _heightsFile, _hazardFile;
        private bool _frozen;

        public int Width { get => _width; set { Check(); _width = value; } }
        public int Height { get => _height; set { Check(); _height = value; } }

        /// <summary>Projection X of the southwest corner.</summary>
        public double OriginX { get => _originX; set { Check(); _originX = value; } }

        /// <summary>Projection Y of the southwest corner.</summary>
        public double OriginY { get => _originY; set { Check(); _originY = value; } }

        /// <summary>Side of the square in meters (edge to edge of the grid).</summary>
        public double SideM { get => _sideM; set { Check(); _sideM = value; } }

        public double ZMin { get => _zMin; set { Check(); _zMin = value; } }
        public double ZMax { get => _zMax; set { Check(); _zMax = value; } }
        public string Projection { get => _projection; set { Check(); _projection = value; } }
        public string HeightsFile { get => _heightsFile; set { Check(); _heightsFile = value; } }
        public string HazardFile { get => _hazardFile; set { Check(); _hazardFile = value; } }

        /// <summary>Distance between neighboring nodes. With 1025 nodes over 2000 m: 1.953125 m (same as Unity).</summary>
        public double CellM => SideM / (Width - 1);

        /// <summary>True once the metadata belongs to a map and can no longer change.</summary>
        public bool IsFrozen => _frozen;

        /// <summary>Converts projection X, Y to local meters (x east, z north, SW origin).</summary>
        public Point2 ProjectionToLocal(double x, double y) => new Point2(x - OriginX, y - OriginY);

        /// <summary>Converts local meters to projection X, Y.</summary>
        public (double X, double Y) LocalToProjection(Point2 p) => (p.X + OriginX, p.Z + OriginY);

        /// <summary>An editable copy (for building another map from this one).</summary>
        public TerrainMetadata Clone() => Copy(false);

        internal TerrainMetadata FrozenCopy() => Copy(true);

        private TerrainMetadata Copy(bool frozen) => new TerrainMetadata
        {
            _width = _width, _height = _height, _originX = _originX, _originY = _originY, _sideM = _sideM,
            _zMin = _zMin, _zMax = _zMax, _projection = _projection, _heightsFile = _heightsFile,
            _hazardFile = _hazardFile, _frozen = frozen,
        };

        private void Check()
        {
            if (_frozen)
                throw new InvalidOperationException("This metadata belongs to a map and cannot be changed; use Clone() to build another map.");
        }
    }
}
