using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using Marps.Engine.Abstractions;
using Marps.Engine.Mission;
using Marps.Engine.Navigation;
using Marps.Engine.Terrain;
using Xunit;
using static Marps.Engine.Tests.Interface.Fixtures;

namespace Marps.Engine.Tests.Interface
{
    [Trait("Method", "Map / Mission / Base")]
    public class PropertiesEdgeCaseTests
    {
        [Fact]
        public void MapAndMission_AreTheInstancesGiven()
        {
            var map = Flat();
            var mission = Square();
            using INavigationEngine engine = new NavigationEngine(map, mission);
            Assert.Same(map, engine.Map);
            Assert.Same(mission, engine.Mission);
        }

        [Fact]
        public void Base_IsTheDepartureStepWhenItIsABase()
        {
            Assert.Equal("B", NewEngine().Base.Id);
        }

        [Fact]
        public void Base_FallsBackToTheReturnStep()
        {
            var places = new[] { Wp("P1", "science", 60, 60), Wp("HOME", "base", 20, 20) };
            Assert.Equal("HOME", NewEngine(mission: Custom(places, "P1", "HOME")).Base.Id);
        }

        [Fact]
        public void Base_FallsBackToAnyBasePlace()
        {
            var places = new[] { Wp("P1", "science", 60, 60), Wp("HAB", "base", 20, 20) };
            Assert.Equal("HAB", NewEngine(mission: Custom(places, "P1")).Base.Id);
        }

        [Fact]
        public void Base_PrefersTheDepartureOverOtherBases()
        {
            var places = new[] { Wp("ALT", "base", 100, 100), Wp("B", "base", 10, 10), Wp("P1", "science", 60, 60) };
            Assert.Equal("B", NewEngine(mission: Custom(places, "B", "P1")).Base.Id);
        }

        [Fact]
        public void Base_IsNullWithoutAnyBase()
        {
            Assert.Null(NewEngine(mission: Custom(new[] { Wp("P1", "science", 60, 60) }, "P1")).Base);
        }

        [Fact]
        public void EmptyMission_HasNoBase()
        {
            Assert.Null(NewEngine(mission: Custom(new Waypoint[0])).Base);
        }

        [Fact]
        public void Constructor_RejectsNullMapOrMission()
        {
            Assert.Throws<ArgumentNullException>(() => new NavigationEngine(null, Square()));
            Assert.Throws<ArgumentNullException>(() => new NavigationEngine(Flat(), null));
        }
    }

    [Trait("Method", "Dispose")]
    public class DisposeEdgeCaseTests
    {
        private static readonly NavigationOptions Background = new NavigationOptions { BackgroundReplanning = true };

        [Fact]
        public void SynchronousEngine_CanBeDisposedTwice()
        {
            var engine = NewEngine();
            engine.Dispose();
            engine.Dispose();
        }

        [Fact]
        public void BackgroundEngine_CanBeDisposedTwice()
        {
            var engine = NewEngine(options: Background);
            engine.Dispose();
            engine.Dispose();
        }

        [Fact]
        public void UpdateAfterDispose_StillReturnsARoute()
        {
            var engine = NewEngine(options: Background);
            engine.Dispose();
            var resp = engine.Update(At(BasePos));                     // no worker: plans on the caller's thread
            Assert.True(resp.Route.Found);
            Assert.False(resp.Route.Pending);
        }

        [Fact]
        public void PlanningAfterDispose_StillWorks()
        {
            var engine = NewEngine(options: Background);
            engine.Dispose();
            Assert.True(engine.PlanRoute(BasePos, P1Pos).Found);
            Assert.True(engine.PlanTrip(BasePos, null, P2Pos).Found);
        }

        [Fact]
        public void DisposeWithSearchesQueued_ReturnsPromptly()
        {
            var engine = NewEngine(options: Background);
            for (int i = 0; i < 50; i++) engine.Update(At(10 + i, 10, "q" + i));
            var clock = Stopwatch.StartNew();
            engine.Dispose();
            Assert.True(clock.ElapsedMilliseconds < 6000);
        }

        [Fact]
        public void SessionStateSurvivesDispose()
        {
            var engine = NewEngine(options: Background);
            Settle(engine, At(BasePos));
            Settle(engine, At(P1Pos));
            engine.Dispose();
            Assert.Equal("P2", engine.Update(At(P1Pos)).NextWaypoint.Id);
        }
    }

    /// <summary>The frontend interfaces as seen by the engine loop (<see cref="NavigationService"/>).</summary>
    [Trait("Method", "IAstronautSensor.Read")]
    public class SensorEdgeCaseTests
    {
        private sealed class Sensor : IAstronautSensor
        {
            public Func<NavigationRequest> Next;
            public NavigationRequest Read() => Next();
        }

        [Fact]
        public void EmergencyButtonFromTheSensor_ReachesTheEngine()
        {
            var req = At(60, 60);
            req.EmergencyReturn = true;
            var service = new NavigationService(NewEngine(), new Sensor { Next = () => req }, null);
            var resp = service.Step();
            Assert.True(resp.Emergency.Active);
        }

        [Fact]
        public void StopOrderFromTheSensor_ReachesTheEngine()
        {
            var req = At(BasePos);
            req.StopOrder = new List<string> { "P3" };
            var resp = new NavigationService(NewEngine(), new Sensor { Next = () => req }, null).Step();
            Assert.Equal("P3", resp.NextWaypoint.Id);
        }

        [Theory]
        [InlineData(double.NaN)]
        [InlineData(double.PositiveInfinity)]
        public void NonFiniteHeading_IsRejectedAsASensorFault(double heading)
        {
            var service = new NavigationService(NewEngine(), new Sensor { Next = () => At(10, 10, heading: heading) }, null);
            var resp = service.Step();
            Assert.Contains(resp.Alerts, a => a.Code == "SENSOR_FAULT");
            Assert.Equal(1, service.SensorFaults);
        }

        [Fact]
        public void IntermittentSensor_RecoversOnTheNextGoodReading()
        {
            int calls = 0;
            var service = new NavigationService(NewEngine(), new Sensor
            {
                Next = () => ++calls % 2 == 0 ? throw new IOException("link down") : At(BasePos),
            }, null);
            for (int i = 0; i < 6; i++) service.Step();
            Assert.Equal(3, service.SensorFaults);
            Assert.Equal("link down", service.LastError);
            Assert.NotNull(service.Step().Route);
        }
    }

    [Trait("Method", "IHudSink.Publish")]
    public class SinkEdgeCaseTests
    {
        private sealed class Sink : IHudSink
        {
            public readonly List<NavigationResponse> Got = new List<NavigationResponse>();
            public void Publish(NavigationResponse response) => Got.Add(response);
        }

        private sealed class FixedSensor : IAstronautSensor
        {
            public NavigationRequest Value;
            public NavigationRequest Read() => Value;
        }

        [Fact]
        public void NoSink_IsAllowed()
        {
            var resp = new NavigationService(NewEngine(), new FixedSensor { Value = At(BasePos) }, null).Step();
            Assert.NotNull(resp.Route);
        }

        [Fact]
        public void SinkGetsExactlyTheReturnedResponse()
        {
            var sink = new Sink();
            var resp = new NavigationService(NewEngine(), new FixedSensor { Value = At(BasePos) }, sink).Step();
            Assert.Same(resp, sink.Got.Single());
        }

        [Fact]
        public void SinkIsNotifiedOfSensorFaultsToo()
        {
            var sink = new Sink();
            new NavigationService(NewEngine(), new FixedSensor { Value = null }, sink).Step();
            Assert.Contains(sink.Got.Single().Alerts, a => a.Code == "SENSOR_FAULT");
        }

        [Fact]
        public void EmergencyResponse_IsPublished()
        {
            var sink = new Sink();
            var req = At(60, 60);
            req.EmergencyReturn = true;
            new NavigationService(NewEngine(), new FixedSensor { Value = req }, sink).Step();
            Assert.NotNull(sink.Got.Single().Emergency);
        }
    }

    [Trait("Method", "ISpaceWeatherProvider.Current")]
    public class WeatherEdgeCaseTests
    {
        private sealed class Weather : ISpaceWeatherProvider
        {
            public SpaceWeather Value;
            public SpaceWeather Current() => Value;
        }

        private sealed class FixedSensor : IAstronautSensor
        {
            public NavigationRequest Read() => At(BasePos);
        }

        [Fact]
        public void NullWeather_IsNotAFault()
        {
            var service = new NavigationService(NewEngine(), new FixedSensor(), null, new Weather { Value = null });
            var resp = service.Step();
            Assert.Equal(0, service.WeatherFaults);
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "SOLAR_STORM" || a.Code == "WEATHER_UNAVAILABLE");
        }

        [Fact]
        public void ActiveStorm_RaisesTheAlert_WithItsLevel()
        {
            var weather = new Weather { Value = new SpaceWeather { StormActive = true, Level = "G4", Source = "test" } };
            var resp = new NavigationService(NewEngine(), new FixedSensor(), null, weather).Step();
            Assert.Contains(resp.Alerts, a => a.Code == "SOLAR_STORM" && a.Message.Contains("G4"));
        }

        [Fact]
        public void StormWithoutLevel_StillRaisesTheAlert()
        {
            var weather = new Weather { Value = new SpaceWeather { StormActive = true } };
            var resp = new NavigationService(NewEngine(), new FixedSensor(), null, weather).Step();
            Assert.Contains(resp.Alerts, a => a.Code == "SOLAR_STORM" && a.Message.Contains("no level"));
        }

        [Fact]
        public void QuietWeather_RaisesNothing()
        {
            var weather = new Weather { Value = new SpaceWeather { StormActive = false, Level = "G0" } };
            var resp = new NavigationService(NewEngine(), new FixedSensor(), null, weather).Step();
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "SOLAR_STORM");
        }
    }

    [Trait("Method", "ITerrainProvider.Load")]
    public class TerrainProviderEdgeCaseTests
    {
        [Fact]
        public void MissingDirectory_FailsWithAClearError()
        {
            var provider = new FileTerrainProvider(Path.Combine(Path.GetTempPath(), "marps-missing-" + Guid.NewGuid().ToString("N")));
            var ex = Assert.ThrowsAny<Exception>(() => provider.Load());
            Assert.Contains("terrain", ex.Message, StringComparison.OrdinalIgnoreCase);
        }

        [Fact]
        public void CustomProvider_FeedsTheEngine()
        {
            ITerrainProvider provider = new InMemoryTerrain();
            using INavigationEngine engine = new NavigationEngine(provider.Load(), Square());
            Assert.True(engine.PlanRoute(BasePos, P1Pos).Found);
        }

        private sealed class InMemoryTerrain : ITerrainProvider
        {
            public TerrainMap Load() => Flat();
        }
    }
}
