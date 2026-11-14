using Marps.Engine.Abstractions;

namespace Marps.Engine.Navigation
{
    /// <summary>
    /// Connects sensors and outputs to the engine: reads the astronaut, queries weather,
    /// computes and publishes the HUD. Each frontend (Unity, web, hardware) only implements interfaces.
    /// </summary>
    public sealed class NavigationService
    {
        private readonly NavigationEngine _engine;
        private readonly IAstronautSensor _sensor;
        private readonly IHudSink _sink;
        private readonly ISpaceWeatherProvider _weather;

        public NavigationService(NavigationEngine engine, IAstronautSensor sensor, IHudSink sink,
            ISpaceWeatherProvider weather = null)
        {
            _engine = engine;
            _sensor = sensor;
            _sink = sink;
            _weather = weather;
        }

        public NavigationResponse Step()
        {
            NavigationRequest request = _sensor.Read();
            if (_weather != null) request.SpaceWeather = _weather.Current();
            NavigationResponse response = _engine.Update(request);
            _sink?.Publish(response);
            return response;
        }
    }
}
