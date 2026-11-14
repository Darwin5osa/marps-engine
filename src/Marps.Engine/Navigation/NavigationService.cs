using System;
using Marps.Engine.Abstractions;

namespace Marps.Engine.Navigation
{
    /// <summary>
    /// Connects sensors and outputs to the engine: reads the astronaut, queries weather,
    /// computes and publishes the HUD. Each frontend (Unity, web, hardware) only implements interfaces.
    ///
    /// Faults in the sensor, the weather provider or the HUD sink do not stop the loop. Each one is counted,
    /// reported as an alert in the response, and the service keeps running. A missing or invalid sensor reading
    /// yields a response that carries a SENSOR_FAULT alert and no route.
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
            _engine = engine ?? throw new ArgumentNullException(nameof(engine));
            _sensor = sensor ?? throw new ArgumentNullException(nameof(sensor));
            _sink = sink;
            _weather = weather;
        }

        /// <summary>Number of steps where the sensor threw or returned an invalid reading.</summary>
        public int SensorFaults { get; private set; }

        /// <summary>Number of steps where the weather provider threw.</summary>
        public int WeatherFaults { get; private set; }

        /// <summary>Number of HUD publications that threw.</summary>
        public int SinkFaults { get; private set; }

        /// <summary>Message of the most recent fault, or null if none has happened.</summary>
        public string LastError { get; private set; }

        public NavigationResponse Step()
        {
            NavigationRequest request;
            try
            {
                request = _sensor.Read();
            }
            catch (Exception ex)
            {
                return Fault("SENSOR_FAULT", "Astronaut sensor unavailable.", ex, sensor: true);
            }

            if (request == null || !IsValid(request))
                return Fault("SENSOR_FAULT", "Astronaut sensor returned an invalid reading.", null, sensor: true);

            string weatherAlert = null;
            if (_weather != null)
            {
                try
                {
                    request.SpaceWeather = _weather.Current();
                }
                catch (Exception ex)
                {
                    WeatherFaults++;
                    LastError = ex.Message;
                    request.SpaceWeather = null;
                    weatherAlert = "Space weather unavailable.";
                }
            }

            NavigationResponse response = _engine.Update(request);
            if (weatherAlert != null)
                response.Alerts.Add(new AlertInfo { Code = "WEATHER_UNAVAILABLE", Severity = "info", Message = weatherAlert });

            if (_sink != null)
            {
                try
                {
                    _sink.Publish(response);
                }
                catch (Exception ex)
                {
                    // The engine state is already updated; a broken display must not stop navigation.
                    SinkFaults++;
                    LastError = ex.Message;
                }
            }
            return response;
        }

        private NavigationResponse Fault(string code, string message, Exception ex, bool sensor)
        {
            if (sensor) SensorFaults++;
            LastError = ex?.Message ?? message;

            var response = new NavigationResponse
            {
                SessionId = "default",
                Alerts = { new AlertInfo { Code = code, Severity = "danger", Message = message } },
            };
            try
            {
                _sink?.Publish(response);
            }
            catch (Exception)
            {
                SinkFaults++;
            }
            return response;
        }

        private static bool IsValid(NavigationRequest r) =>
            double.IsFinite(r.X) && double.IsFinite(r.Z) && double.IsFinite(r.HeadingDeg);
    }
}
