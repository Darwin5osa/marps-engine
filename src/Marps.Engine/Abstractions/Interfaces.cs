using Marps.Engine.Navigation;
using Marps.Engine.Terrain;

namespace Marps.Engine.Abstractions
{
    /// <summary>
    /// Source of the astronaut state: Quest headset (Unity), web simulator, physical sensors, etc.
    /// The engine does not know where the data comes from.
    /// </summary>
    public interface IAstronautSensor
    {
        NavigationRequest Read();
    }

    /// <summary>HUD destination: Unity, a test viewer, a physical panel, etc.</summary>
    public interface IHudSink
    {
        void Publish(NavigationResponse response);
    }

    /// <summary>Space weather provider (DONKI or another source).</summary>
    public interface ISpaceWeatherProvider
    {
        SpaceWeather Current();
    }

    /// <summary>Terrain map provider (processed files, streaming, etc.).</summary>
    public interface ITerrainProvider
    {
        TerrainMap Load();
    }
}
